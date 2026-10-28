"""ROS-independent core of the shared-control teleoperation pipeline.

Every function here works on plain numpy vectors in the robot base frame, so
the same code runs in the offline simulator, in the ROS 2 / Gazebo nodes and
on the real Franka Panda.
"""

from .estimator import ConstraintEstimator
from .assistance import k_perp, assist
from .haptics import force_split, rumble, trigger_resistance
from .teleop_mapping import deadzone, user_velocity
from .params import Params

__all__ = [
    'ConstraintEstimator', 'k_perp', 'assist', 'force_split', 'rumble',
    'trigger_resistance', 'deadzone', 'user_velocity', 'Params',
]
