import math

from stretch4_mission.mission_controller import pose_error


def test_pose_error():
    assert pose_error(0.0, 0.0, 0.0, 3.0, 4.0, 0.0) == (5.0, 0.0)
    # yaw wraps across +/-pi: 179 deg vs -179 deg is 2 deg apart, not 358
    _, dyaw = pose_error(0, 0, math.radians(179), 0, 0, math.radians(-179))
    assert math.isclose(dyaw, math.radians(2), abs_tol=1e-9)
    # sign = rotation needed: from the failed run, yaw -0.575 to target -0.927 is clockwise
    _, dyaw = pose_error(0, 0, -0.575, 0, 0, -0.927)
    assert math.isclose(dyaw, -0.352, abs_tol=1e-9)
