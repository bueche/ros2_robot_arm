"""Make the package importable when pytest runs from the source tree.

Lets the ROS-free tests run with plain pytest from any directory, e.g. from
the repository root:  python3 -m pytest robot_arm_control/test/test_balance_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
