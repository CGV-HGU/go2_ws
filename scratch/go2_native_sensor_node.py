#!/usr/bin/env python3
"""Deprecated compatibility entry point.

The previous implementation relabeled an odom-frame deskewed cloud as a local
sensor frame and also contained a /cmd_vel actuation path.  Keep this filename
for old scripts, but route it to the sensor-only LIVO bridge.
"""

from go2_livo_sensor_bridge import main


if __name__ == '__main__':
    main()
