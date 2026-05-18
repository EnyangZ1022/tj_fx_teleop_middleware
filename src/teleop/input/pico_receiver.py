"""
Pico 位姿接收器
  - UDP 广播（端口 29888）：每 5 秒向子网广播，让 Pico 自动发现本机
  - TCP 服务器（端口 63901）：接收 Pico 连接，解析每帧数据

输出内容：
  - 头部位姿 (x, y, z, qx, qy, qz, qw)
  - 左/右手柄位姿
  - 左/右扳机状态（按下=1，未按=0）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
帧数据结构（stateJson 解析后的 dict）：
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

frame = {
    'device_id'    : str,    # Pico 设备 SN 编号，连接握手时获取
    'timestamp_ns' : int,    # 帧时间戳，单位纳秒（Pico 本地时钟）

    # ── 头部 ──────────────────────────────────────────────
    'head_pose' : (x, y, z, qx, qy, qz, qw),
    #   x/y/z  : 头部在世界坐标系中的位置，单位米（左手系，X 轴朝左）
    #   qx~qw  : 四元数表示朝向，qw 为实部

    # ── 手柄（左/右各一份，key 为 left_ctrl / right_ctrl）─
    'left_ctrl' : {
        'pose'             : (x, y, z, qx, qy, qz, qw),  # 手柄位姿，同上坐标系
        'trigger'          : float,   # 食指扳机，范围 [0.0, 1.0]，>0.5 视为按下
        'grip'             : float,   # 握把键，范围 [0.0, 1.0]
        'axis_x'           : float,   # 摇杆 X 轴，范围 [-1.0, 1.0]，右为正
        'axis_y'           : float,   # 摇杆 Y 轴，范围 [-1.0, 1.0]，上为正
        'primary_button'   : bool,    # A 键（右手柄）/ X 键（左手柄）
        'secondary_button' : bool,    # B 键（右手柄）/ Y 键（左手柄）
        'menu_button'      : bool,    # 菜单键（左手柄特有）
    },
    'right_ctrl' : { ... },           # 结构与 left_ctrl 完全一致

    # ── 注意 ──────────────────────────────────────────────
    # 手部追踪（Hand）、身体追踪（Body）、运动捕捉器（Motion）
    # 当前版本未解析，如需使用参考 pico_parser.h 中对应字段
}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TCP 消息帧结构（二进制协议，小端序）：
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  ┌────────┬────────┬──────────────┬─────────────────┬───────────────────────┐
  │ head   │ cmd    │ length       │ payload         │ timestamp  │ tail     │
  │ u8     │ u8     │ u32 LE       │ length bytes    │ u64 LE     │ u8=0xA5  │
  │ 1 byte │ 1 byte │ 4 bytes      │ 0~65535 bytes   │ 8 bytes    │ 1 byte   │
  └────────┴────────┴──────────────┴─────────────────┴───────────────────────┘
  ← Header (6 bytes) ────────────────────────────────────────────────────────→
                                                      ← Tail (9 bytes) ──────→

  Pico→服务器: head = 0x3F
  服务器→Pico: head = 0xCF
  tail magic : 0xA5

  主要 cmd 值：
    0x19  CMD_CONNECT    : Pico 握手，payload = "SN|设备序列号"
    0x23  CMD_HEARTBEAT  : 心跳保活，无 payload
    0x6D  CMD_STATE_JSON : 帧数据，payload = JSON 字符串
    0x7E  CMD_TCPIP      : UDP 广播，payload = 服务器 IP 字符串

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
stateJson 原始格式（CMD_STATE_JSON 的 payload）：
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  外层：{"value": "<内层 JSON 字符串（再次 JSON 编码）>"}

  内层解码后结构：
  {
    "timeStampNs": 1234567890,       # 帧时间戳（纳秒）
    "Head": {
      "pose": "x,y,z,qx,qy,qz,qw"  # 逗号分隔的 7 个浮点数
    },
    "Controller": {
      "left": {
        "pose"           : "x,y,z,qx,qy,qz,qw",
        "trigger"        : 0.0,      # [0,1]
        "grip"           : 0.0,      # [0,1]
        "axisX"          : 0.0,      # [-1,1]
        "axisY"          : 0.0,      # [-1,1]
        "primaryButton"  : false,    # A/X 键
        "secondaryButton": false,    # B/Y 键
        "menuButton"     : false,    # 菜单键
        "axisClick"      : false     # 摇杆按下
      },
      "right": { ... }               # 同 left
    },
    "Hand": {                        # 手部关节追踪（需 Pico 开启手部追踪）
      "leftHand" : {
        "isActive"         : 1,
        "scale"            : 1.0,
        "HandJointLocations": [      # 26 个手部关节
          {"p": "x,y,z,qx,qy,qz,qw"}, ...
        ]
      },
      "rightHand": { ... }
    },
    "Body": {                        # 身体关节追踪（需配套硬件）
      "timeStampNs": ...,
      "joints": [                    # 最多 24 个身体关节
        {
          "p"  : "x,y,z,qx,qy,qz,qw",  # 位姿
          "va" : "vx,vy,vz,wx,wy,wz",   # 速度（线速度+角速度）
          "wva": "ax,ay,az,αx,αy,αz",   # 加速度
          "t"  : 1234567890              # 关节时间戳（纳秒）
        }, ...
      ]
    },
    "Motion": {                      # 运动捕捉器（外接追踪器）
      "timeStampNs": ...,
      "joints": [
        {
          "sn" : "TRACKER_SN",           # 追踪器序列号
          "p"  : "x,y,z,qx,qy,qz,qw",
          "va" : "vx,vy,vz,wx,wy,wz",
          "wva": "ax,ay,az,αx,αy,αz"
        }, ...
      ]
    }
  }
"""

import json
import socket
import struct
import threading
import time

# ─── 协议常量 ──────────────────────────────────────────────────────────────────
PICO_HEAD   = 0x3F   # Pico → 服务器的消息起始字节
SERVER_HEAD = 0xCF   # 服务器 → Pico 的消息起始字节
MSG_TAIL    = 0xA5   # 所有消息的结束魔数

CMD_TCPIP      = 0x7E  # UDP 广播：告知 Pico 服务器 IP
CMD_CONNECT    = 0x19  # Pico 发来：连接握手，payload="SN|设备序列号"
CMD_HEARTBEAT  = 0x23  # Pico 发来：心跳保活，无 payload
CMD_STATE_JSON = 0x6D  # Pico 发来：每帧位姿数据（JSON 格式）

TCP_PORT       = 63901  # TCP 服务器端口，与 XRoboToolkit PC Service 保持一致
UDP_BCAST_PORT = 29888  # UDP 广播端口，Pico App 监听此端口发现服务器

# 消息头：head(u8 1字节) + cmd(u8 1字节) + length(u32LE 4字节) = 6字节
# 消息尾：timestamp(u64LE 8字节) + tail(u8 1字节) = 9字节
HEAD_FMT = struct.Struct('<BBL')   # little-endian: u8 u8 u32
TAIL_FMT = struct.Struct('<QB')    # little-endian: u64 u8
HEAD_SZ  = HEAD_FMT.size           # 6 字节
TAIL_SZ  = TAIL_FMT.size           # 9 字节


# ─── 工具函数 ──────────────────────────────────────────────────────────────────

def local_ipv4():
    """获取本机所有非 loopback IPv4 地址，用于 UDP 广播"""
    ips = []
    try:
        hostname = socket.gethostname()
        infos = socket.getaddrinfo(hostname, None, socket.AF_INET)
        for info in infos:
            ip = info[4][0]
            if not ip.startswith('127.'):
                ips.append(ip)
    except Exception:
        pass
    if not ips:
        # 无法通过 hostname 获取时的 fallback：连接外网获取出口 IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ips.append(s.getsockname()[0])
            s.close()
        except Exception:
            pass
    return ips


def to_bcast(ip: str) -> str:
    """将 IP 最后一段替换为 255，得到子网广播地址。如 192.168.1.5 → 192.168.1.255"""
    parts = ip.rsplit('.', 1)
    return parts[0] + '.255' if len(parts) == 2 else ip


def build_server_msg(cmd: int, payload: bytes) -> bytes:
    """构建服务器→Pico 的标准消息帧（Header + Payload + Tail）"""
    length = len(payload)
    header = HEAD_FMT.pack(SERVER_HEAD, cmd, length)
    tail   = TAIL_FMT.pack(int(time.time()), MSG_TAIL)
    return header + payload + tail


def parse_pose_str(s: str) -> tuple:
    """
    解析 Pico 位姿字符串 → 7元组
    输入格式："x,y,z,qx,qy,qz,qw"（逗号分隔，左手坐标系）
    输出：(x, y, z, qx, qy, qz, qw)
    """
    parts = s.split(',')
    vals = [float(p) for p in parts]
    while len(vals) < 7:
        vals.append(0.0)
    return tuple(vals[:7])


def parse_state_json(raw_json: str, dev_id: str) -> dict | None:
    """
    解析一帧 stateJson 数据，返回结构化 dict。

    stateJson 是双层 JSON：外层 {"value": "<内层JSON字符串>"}
    内层包含 Head / Controller / Hand / Body / Motion 等字段。

    返回的 frame dict 结构见文件顶部文档。
    解析失败返回 None。
    """
    try:
        # 第一层解析：拿到 value 字段
        outer = json.loads(raw_json)
        inner_str = outer.get('value', '')

        # 第二层解析：拿到实际帧数据
        v = json.loads(inner_str)

        frame = {
            'device_id'   : dev_id,
            'timestamp_ns': v.get('timeStampNs', 0),  # 帧时间戳，单位纳秒
        }

        # ── 头部位姿 ──────────────────────────────────────────────────────────
        # 对应 JSON: "Head": {"pose": "x,y,z,qx,qy,qz,qw"}
        head = v.get('Head', {})
        if 'pose' in head:
            frame['head_pose'] = parse_pose_str(head['pose'])

        # ── 手柄数据 ──────────────────────────────────────────────────────────
        # 对应 JSON: "Controller": {"left": {...}, "right": {...}}
        ctrl = v.get('Controller', {})
        for side in ('left', 'right'):
            c = ctrl.get(side, {})
            frame[f'{side}_ctrl'] = {
                # 手柄在世界坐标系中的绝对位姿（左手系，X轴朝左）
                'pose'            : parse_pose_str(c['pose']) if 'pose' in c else None,
                # 食指扳机深度 [0.0~1.0]，>0.5 判定为按下
                'trigger'         : float(c.get('trigger', 0)),
                # 握把键深度 [0.0~1.0]
                'grip'            : float(c.get('grip', 0)),
                # 摇杆 X 轴 [-1.0~1.0]，右为正；右手柄用于底盘转向
                'axis_x'          : float(c.get('axisX', 0)),
                # 摇杆 Y 轴 [-1.0~1.0]，上为正；右手柄用于底盘前进
                'axis_y'          : float(c.get('axisY', 0)),
                # A键（右手柄）/ X键（左手柄）：遥操 开始/暂停 的触发键
                'primary_button'  : bool(c.get('primaryButton', False)),
                # B键（右手柄）/ Y键（左手柄）
                'secondary_button': bool(c.get('secondaryButton', False)),
                # 菜单键（左手柄特有）
                'menu_button'     : bool(c.get('menuButton', False)),
            }

        return frame
    except Exception:
        return None


def print_frame(frame: dict):
    """格式化打印一帧数据到终端"""
    ts_s = frame['timestamp_ns'] / 1e9
    print(f"\n── device={frame['device_id']}  ts={ts_s:.3f}s ──────────────────")

    # 头部位姿
    head = frame.get('head_pose')
    if head:
        print(f"  Head       pos({head[0]:7.3f} {head[1]:7.3f} {head[2]:7.3f})"
              f"  q({head[3]:6.3f} {head[4]:6.3f} {head[5]:6.3f} {head[6]:6.3f})")

    # 左右手柄
    for side in ('left', 'right'):
        c = frame.get(f'{side}_ctrl', {})
        p = c.get('pose')
        # trigger > 0.5 视为按下，输出 1；否则 0
        trigger_flag = 1 if c.get('trigger', 0) > 0.8 else 0
        if p:
            print(f"  {side.capitalize():5s} ctrl  pos({p[0]:7.3f} {p[1]:7.3f} {p[2]:7.3f})"
                  f"  q({p[3]:6.3f} {p[4]:6.3f} {p[5]:6.3f} {p[6]:6.3f})"
                  f"  trigger={trigger_flag}"          # 扳机：0=未按，1=按下
                  f"  grip={c['grip']:.2f}"            # 握把深度
                  f"  A/X={int(c['primary_button'])}"  # 主键状态
                  f"  joy=({c['axis_x']:5.2f},{c['axis_y']:5.2f})")  # 摇杆


# ─── UDP 广播线程 ──────────────────────────────────────────────────────────────

def udp_broadcast_loop(stop_event: threading.Event):
    """
    每 5 秒向本机所有网卡的子网广播一次服务器 IP。
    Pico App 收到广播后会主动发起 TCP 连接。
    协议与 XRoboToolkit PC Service 完全兼容，Pico App 无需修改任何配置。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.1)
    print(f'[UDP] broadcaster started, port={UDP_BCAST_PORT}')

    while not stop_event.is_set():
        for ip in local_ipv4():
            payload = ip.encode()                        # payload = 本机 IP 字符串
            msg     = build_server_msg(CMD_TCPIP, payload)
            bcast   = to_bcast(ip)                       # 计算广播地址
            try:
                sock.sendto(msg, (bcast, UDP_BCAST_PORT))
            except Exception:
                pass
        # 5 秒 = 50 × 100ms，拆小片段方便快速响应退出信号
        for _ in range(50):
            if stop_event.is_set():
                break
            time.sleep(0.1)

    sock.close()
    print('[UDP] broadcaster stopped')


# ─── TCP 流式消息解析器 ────────────────────────────────────────────────────────

class PicoStreamParser:
    """
    字节级状态机，解决 TCP 粘包/拆包问题。

    TCP 是流协议，单次 recv() 可能包含半条消息或多条消息。
    状态机逐字节扫描，按 Header→Body→Tail 顺序累积，
    拿到完整一帧后再调用 on_frame 回调。

    状态转移：
      WAIT_HEAD : 等待 0x3F 起始字节
      READ_HEAD : 累积 6 字节 Header，解出 cmd 和 payload 长度
      READ_BODY : 累积 length 字节 payload
      READ_TAIL : 累积 9 字节 Tail，校验 0xA5 尾字节，然后 dispatch
    """

    WAIT_HEAD = 0
    READ_HEAD = 1
    READ_BODY = 2
    READ_TAIL = 3

    def __init__(self, conn: socket.socket, addr, on_frame):
        self.conn     = conn
        self.addr     = addr
        self.on_frame = on_frame  # 回调：on_frame(frame_dict)
        self.dev_id   = ''        # 设备 SN，握手时赋值
        self._reset()

    def _reset(self):
        """清空缓冲区，回到等待起始字节状态"""
        self.buf        = bytearray()
        self.state      = self.WAIT_HEAD
        self.hdr_cmd    = 0
        self.hdr_length = 0

    def run(self):
        """持续接收数据直到连接断开"""
        self.conn.settimeout(2.0)
        try:
            while True:
                try:
                    chunk = self.conn.recv(8192)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                for b in chunk:
                    self._feed(b)
        except Exception:
            pass
        self.conn.close()
        print(f'[TCP] {self.addr} disconnected')

    def _feed(self, b: int):
        """喂入一个字节，驱动状态机"""
        self.buf.append(b)

        if self.state == self.WAIT_HEAD:
            # 等待 Pico→服务器的起始字节 0x3F
            if b == PICO_HEAD:
                self.state = self.READ_HEAD
            else:
                self.buf.clear()  # 不是起始字节，丢掉重来

        elif self.state == self.READ_HEAD:
            # 累积到 6 字节后解析 Header
            if len(self.buf) == HEAD_SZ:
                _, self.hdr_cmd, self.hdr_length = HEAD_FMT.unpack_from(self.buf)
                if self.hdr_length > 65535:      # 超长包异常保护
                    self._reset()
                    return
                # payload 为空时直接跳到读 Tail
                self.state = self.READ_BODY if self.hdr_length > 0 else self.READ_TAIL

        elif self.state == self.READ_BODY:
            # 累积完整 payload
            if len(self.buf) == HEAD_SZ + self.hdr_length:
                self.state = self.READ_TAIL

        elif self.state == self.READ_TAIL:
            # 累积完整 Tail（9字节），校验尾魔数后分发
            if len(self.buf) == HEAD_SZ + self.hdr_length + TAIL_SZ:
                if self.buf[-1] == MSG_TAIL:     # 校验 0xA5
                    self._dispatch()
                self._reset()

    def _dispatch(self):
        """根据 cmd 类型处理完整消息"""
        payload = bytes(self.buf[HEAD_SZ: HEAD_SZ + self.hdr_length])

        if self.hdr_cmd == CMD_CONNECT:
            # 握手消息：payload = "SN|设备序列号"，取 | 前的部分作为设备 ID
            msg = payload.decode(errors='replace')
            self.dev_id = msg.split('|')[0]
            print(f'[TCP] Device connected: {self.dev_id}')

        elif self.hdr_cmd == CMD_HEARTBEAT:
            pass  # 心跳不处理，连接保活由 TCP 层自动维持

        elif self.hdr_cmd == CMD_STATE_JSON:
            # 帧数据：payload 是 stateJson 字符串，解析后回调
            if not payload:
                return
            raw   = payload.decode(errors='replace')
            frame = parse_state_json(raw, self.dev_id)
            if frame:
                self.on_frame(frame)


def handle_client(conn, addr, on_frame):
    """每个 Pico 连接独立运行在一个线程中"""
    print(f'[TCP] Connection from {addr[0]}')
    parser = PicoStreamParser(conn, addr, on_frame)
    parser.run()


# ─── TCP 服务器线程 ────────────────────────────────────────────────────────────

def tcp_server_loop(stop_event: threading.Event, on_frame):
    """
    监听 TCP 63901 端口，每个新连接起一个独立线程处理。
    select timeout=1s 轮询，以便快速响应退出信号。
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    srv.bind(('', TCP_PORT))
    srv.listen(8)
    srv.settimeout(1.0)
    print(f'[TCP] Listening on port {TCP_PORT}')

    while not stop_event.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        t = threading.Thread(target=handle_client, args=(conn, addr, on_frame), daemon=True)
        t.start()

    srv.close()
    print('[TCP] server stopped')


# ─── 主程序 ───────────────────────────────────────────────────────────────────

def main():
    stop_event = threading.Event()

    # 后台线程：UDP 广播（让 Pico 发现本机）
    threading.Thread(target=udp_broadcast_loop, args=(stop_event,), daemon=True).start()
    # 后台线程：TCP 服务器（接收 Pico 连接和帧数据）
    threading.Thread(target=tcp_server_loop, args=(stop_event, print_frame), daemon=True).start()

    print('Pico receiver started. Waiting for device... (Ctrl+C to quit)')
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nShutting down...')
        stop_event.set()
        time.sleep(0.5)


if __name__ == '__main__':
    main()
