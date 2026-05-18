from teleop.core.pose import Pose7


def test_pose7_from_tuple() -> None:
    pose = Pose7.from_tuple((1, 2, 3, 0, 0, 0, 1))
    assert pose.as_tuple() == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)


def test_zero_pose_is_invalid() -> None:
    pose = Pose7.from_tuple((0, 0, 0, 0, 0, 0, 0))
    assert pose.is_zero_pose()
    assert not pose.is_valid()


def test_valid_quaternion_pose_is_valid() -> None:
    pose = Pose7.from_tuple((0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0))
    assert pose.is_valid()
