from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time
from dataclasses import replace
from typing import Callable

from teleop.core.pico_frame import PicoControllerState, PicoRawFrame
from teleop.core.pose import Pose7

# Protocol constants retained from the customer demo.
PICO_HEAD = 0x3F
SERVER_HEAD = 0xCF
MSG_TAIL = 0xA5

CMD_TCPIP = 0x7E
CMD_CONNECT = 0x19
CMD_HEARTBEAT = 0x23
CMD_STATE_JSON = 0x6D

TCP_PORT = 63901
UDP_BCAST_PORT = 29888

HEAD_FMT = struct.Struct("<BBL")
TAIL_FMT = struct.Struct("<QB")
HEAD_SZ = HEAD_FMT.size
TAIL_SZ = TAIL_FMT.size

OnFrameCallback = Callable[[PicoRawFrame], None]
OnStateJsonCallback = Callable[[str, str], None]
OnReceiverTimingCallback = Callable[[dict[str, object]], None]

LOGGER = logging.getLogger(__name__)


def local_ipv4() -> list[str]:
    """Get non-loopback local IPv4 addresses for UDP broadcast."""
    ips: list[str] = []
    try:
        hostname = socket.gethostname()
        infos = socket.getaddrinfo(hostname, None, socket.AF_INET)
        for info in infos:
            ip = info[4][0]
            if not ip.startswith("127."):
                ips.append(ip)
    except Exception:
        LOGGER.debug("Failed to resolve host IPv4 addresses", exc_info=True)

    if not ips:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("8.8.8.8", 80))
            ips.append(sock.getsockname()[0])
            sock.close()
        except Exception:
            LOGGER.debug("Failed to get fallback outbound IPv4", exc_info=True)

    return ips


def to_bcast(ip: str) -> str:
    parts = ip.rsplit(".", 1)
    return f"{parts[0]}.255" if len(parts) == 2 else ip


def build_server_msg(cmd: int, payload: bytes) -> bytes:
    length = len(payload)
    header = HEAD_FMT.pack(SERVER_HEAD, cmd, length)
    tail = TAIL_FMT.pack(int(time.time()), MSG_TAIL)
    return header + payload + tail


def _to_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_int(value: object, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def parse_pose_str(raw_pose: str) -> Pose7:
    """Parse pose string "x,y,z,qx,qy,qz,qw" into a Pose7 object."""
    values = [float(part) for part in raw_pose.split(",") if part != ""]
    while len(values) < 7:
        values.append(0.0)
    return Pose7.from_tuple(values[:7])


def _parse_controller(ctrl_json: dict[str, object]) -> PicoControllerState:
    pose_raw = ctrl_json.get("pose", "0,0,0,0,0,0,0")
    pose = parse_pose_str(pose_raw if isinstance(pose_raw, str) else "0,0,0,0,0,0,0")

    return PicoControllerState(
        pose=pose,
        trigger=_to_float(ctrl_json.get("trigger", 0.0)),
        grip=_to_float(ctrl_json.get("grip", 0.0)),
        axis_x=_to_float(ctrl_json.get("axisX", 0.0)),
        axis_y=_to_float(ctrl_json.get("axisY", 0.0)),
        axis_click=bool(ctrl_json.get("axisClick", False)),
        primary_button=bool(ctrl_json.get("primaryButton", False)),
        secondary_button=bool(ctrl_json.get("secondaryButton", False)),
        menu_button=bool(ctrl_json.get("menuButton", False)),
    )


def parse_state_json(
    raw_json: str,
    dev_id: str = "",
    frame_id: int = 0,
    pc_receive_time_ns: int | None = None,
) -> PicoRawFrame | None:
    """
    Parse CMD_STATE_JSON payload (double-encoded JSON) into PicoRawFrame.

    Outer JSON contains "value", and "value" is another JSON string.
    """
    try:
        outer = json.loads(raw_json)
        inner_raw = outer.get("value", "")
        if not isinstance(inner_raw, str):
            return None

        inner = json.loads(inner_raw)
        head_json = inner.get("Head", {})
        ctrl_json = inner.get("Controller", {})

        head_pose_raw = "0,0,0,0,0,0,0"
        if isinstance(head_json, dict):
            head_pose_raw = head_json.get("pose", head_pose_raw)
        head_pose = parse_pose_str(head_pose_raw if isinstance(head_pose_raw, str) else "0,0,0,0,0,0,0")

        left_json = ctrl_json.get("left", {}) if isinstance(ctrl_json, dict) else {}
        right_json = ctrl_json.get("right", {}) if isinstance(ctrl_json, dict) else {}

        recv_ts = pc_receive_time_ns if pc_receive_time_ns is not None else time.time_ns()

        return PicoRawFrame(
            frame_id=frame_id,
            device_id=dev_id,
            pico_timestamp_ns=_to_int(inner.get("timeStampNs", 0), 0),
            pc_receive_time_ns=_to_int(recv_ts, 0),
            head_pose=head_pose,
            left_ctrl=_parse_controller(left_json if isinstance(left_json, dict) else {}),
            right_ctrl=_parse_controller(right_json if isinstance(right_json, dict) else {}),
        )
    except Exception:
        LOGGER.debug("Failed to parse CMD_STATE_JSON payload", exc_info=True)
        return None


class PicoStreamParser:
    """Byte stream parser for TCP packet framing (handles sticky/partial packets)."""

    WAIT_HEAD = 0
    READ_HEAD = 1
    READ_BODY = 2
    READ_TAIL = 3

    def __init__(self, conn: socket.socket, addr: tuple[str, int], on_state_json: OnStateJsonCallback):
        self.conn = conn
        self.addr = addr
        self.on_state_json = on_state_json
        self.dev_id = ""
        self._reset()

    def _reset(self) -> None:
        self.buf = bytearray()
        self.state = self.WAIT_HEAD
        self.hdr_cmd = 0
        self.hdr_length = 0

    def run(self) -> None:
        self.conn.settimeout(2.0)
        try:
            while True:
                try:
                    chunk = self.conn.recv(8192)
                except socket.timeout:
                    continue

                if not chunk:
                    break

                for one_byte in chunk:
                    self._feed(one_byte)
        except Exception:
            LOGGER.debug("TCP parser loop exited with error from %s", self.addr, exc_info=True)
        finally:
            try:
                self.conn.close()
            except Exception:
                LOGGER.debug("Failed to close TCP client socket", exc_info=True)

    def _feed(self, one_byte: int) -> None:
        self.buf.append(one_byte)

        if self.state == self.WAIT_HEAD:
            if one_byte == PICO_HEAD:
                self.state = self.READ_HEAD
            else:
                self.buf.clear()

        elif self.state == self.READ_HEAD:
            if len(self.buf) == HEAD_SZ:
                _, self.hdr_cmd, self.hdr_length = HEAD_FMT.unpack_from(self.buf)
                if self.hdr_length > 65535:
                    self._reset()
                    return
                self.state = self.READ_BODY if self.hdr_length > 0 else self.READ_TAIL

        elif self.state == self.READ_BODY:
            if len(self.buf) == HEAD_SZ + self.hdr_length:
                self.state = self.READ_TAIL

        elif self.state == self.READ_TAIL:
            if len(self.buf) == HEAD_SZ + self.hdr_length + TAIL_SZ:
                if self.buf[-1] == MSG_TAIL:
                    self._dispatch()
                self._reset()

    def _dispatch(self) -> None:
        payload = bytes(self.buf[HEAD_SZ : HEAD_SZ + self.hdr_length])

        if self.hdr_cmd == CMD_CONNECT:
            msg = payload.decode(errors="replace")
            self.dev_id = msg.split("|")[0]
            LOGGER.info("Pico device connected: %s", self.dev_id or "<unknown>")

        elif self.hdr_cmd == CMD_HEARTBEAT:
            return

        elif self.hdr_cmd == CMD_STATE_JSON:
            if not payload:
                return
            raw_state = payload.decode(errors="replace")
            self.on_state_json(raw_state, self.dev_id)


def handle_client(conn: socket.socket, addr: tuple[str, int], on_state_json: OnStateJsonCallback) -> None:
    LOGGER.info("TCP client connected from %s:%s", addr[0], addr[1])
    parser = PicoStreamParser(conn, addr, on_state_json)
    parser.run()
    LOGGER.info("TCP client disconnected from %s:%s", addr[0], addr[1])


def tcp_server_loop(stop_event: threading.Event, on_state_json: OnStateJsonCallback) -> None:
    LOGGER.info("Starting TCP server on port %d", TCP_PORT)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    srv.bind(("", TCP_PORT))
    srv.listen(8)
    srv.settimeout(1.0)

    try:
        while not stop_event.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue

            thread = threading.Thread(
                target=handle_client,
                args=(conn, addr, on_state_json),
                daemon=True,
            )
            thread.start()
    finally:
        try:
            srv.close()
        except Exception:
            LOGGER.debug("Failed to close TCP server socket", exc_info=True)
        LOGGER.info("TCP server stopped")


def udp_broadcast_loop(stop_event: threading.Event) -> None:
    LOGGER.info("Starting UDP broadcaster on port %d", UDP_BCAST_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.1)

    try:
        while not stop_event.is_set():
            for ip in local_ipv4():
                payload = ip.encode()
                msg = build_server_msg(CMD_TCPIP, payload)
                bcast = to_bcast(ip)
                try:
                    sock.sendto(msg, (bcast, UDP_BCAST_PORT))
                except Exception:
                    LOGGER.debug("UDP broadcast send failed to %s", bcast, exc_info=True)

            for _ in range(50):
                if stop_event.is_set():
                    break
                time.sleep(0.1)
    finally:
        try:
            sock.close()
        except Exception:
            LOGGER.debug("Failed to close UDP broadcast socket", exc_info=True)
        LOGGER.info("UDP broadcaster stopped")


class PicoReceiver:
    """Receive and parse Pico frames, then deliver them by callback."""

    def __init__(
        self,
        on_frame: OnFrameCallback | None = None,
        on_receiver_timing: OnReceiverTimingCallback | None = None,
    ):
        self._on_frame = on_frame
        self._on_receiver_timing = on_receiver_timing
        self._latest_frame: PicoRawFrame | None = None
        self._frame_id = 0
        self._receiver_seq = 0
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._threads: list[threading.Thread] = []
        self._running = False

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._stop_event.clear()
            udp_thread = threading.Thread(
                target=udp_broadcast_loop,
                args=(self._stop_event,),
                daemon=True,
            )
            tcp_thread = threading.Thread(
                target=tcp_server_loop,
                args=(self._stop_event, self._on_state_json),
                daemon=True,
            )
            self._threads = [udp_thread, tcp_thread]
            self._running = True

        LOGGER.info("PicoReceiver starting")

        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        with self._lock:
            if not self._running:
                return
            self._stop_event.set()
            threads = list(self._threads)
            self._threads = []
            self._running = False

        LOGGER.info("PicoReceiver stopping")

        for thread in threads:
            thread.join(timeout=1.5)

        LOGGER.info("PicoReceiver stopped")

    def get_latest_frame(self) -> PicoRawFrame | None:
        with self._lock:
            return self._latest_frame

    def _on_state_json(self, raw_json: str, dev_id: str) -> None:
        pc_receive_perf_ns = time.perf_counter_ns()
        pc_receive_wall_ns = time.time_ns()

        json_size_bytes: int | None = None
        try:
            if isinstance(raw_json, str):
                json_size_bytes = len(raw_json.encode("utf-8"))
            elif isinstance(raw_json, (bytes, bytearray)):
                json_size_bytes = len(raw_json)
        except Exception:
            json_size_bytes = None

        frame = parse_state_json(
            raw_json=raw_json,
            dev_id=dev_id,
            frame_id=0,
            pc_receive_time_ns=pc_receive_wall_ns,
        )
        parse_done_perf_ns = time.perf_counter_ns()

        callback = None
        receiver_timing_callback = None
        receiver_seq = 0
        frame_seq: int | None = None
        pico_source_timestamp_ns: int | None = None

        with self._lock:
            self._receiver_seq += 1
            receiver_seq = int(self._receiver_seq)

            if frame is not None:
                self._frame_id += 1
                frame = replace(frame, frame_id=self._frame_id)
                self._latest_frame = frame
                frame_seq = int(frame.frame_id)
                pico_source_timestamp_ns = int(frame.pico_timestamp_ns)
            callback = self._on_frame
            receiver_timing_callback = self._on_receiver_timing

        if receiver_timing_callback is not None:
            try:
                receiver_timing_callback(
                    {
                        "receiver_seq": receiver_seq,
                        "pc_receive_perf_ns": int(pc_receive_perf_ns),
                        "pc_receive_wall_ns": int(pc_receive_wall_ns),
                        "pico_source_timestamp_ns": pico_source_timestamp_ns,
                        "frame_seq": frame_seq,
                        "parse_duration_ms": float(parse_done_perf_ns - pc_receive_perf_ns) / 1_000_000.0,
                        "json_size_bytes": json_size_bytes,
                    }
                )
            except Exception:
                LOGGER.debug("PicoReceiver receiver timing callback failed", exc_info=True)

        if frame is None:
            return

        if callback is not None:
            try:
                callback(frame)
            except Exception:
                LOGGER.exception("PicoReceiver on_frame callback failed")


__all__ = [
    "PICO_HEAD",
    "SERVER_HEAD",
    "MSG_TAIL",
    "CMD_TCPIP",
    "CMD_CONNECT",
    "CMD_HEARTBEAT",
    "CMD_STATE_JSON",
    "TCP_PORT",
    "UDP_BCAST_PORT",
    "parse_pose_str",
    "parse_state_json",
    "PicoReceiver",
]
