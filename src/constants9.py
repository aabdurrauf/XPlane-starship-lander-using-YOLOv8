# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 17:03:49 2024

@author: Ammar Abdurrauf
"""

# ---------------------------------------
# XPlane9.py

# directory
FLIGTH_DATA_DIR = 'D:\\Projects\\xplane_yolo\\flight_data\\'
FLIGTH_DATA_FILENAME = 'starship_landing_gps_flight_data'
CONTROL_DATA_FILENAME = 'starship_landing_gps_control_data'

CENTER_X = 639
CENTER_Y = 226
# coordinate of the landing area (sometimes change)
LANDING_CENTER_X = -15531.15 # -15526.20
LANDING_CENTER_Z = -55350.81 # 55967.3125

INITIAL_POS_X = -15728
INITIAL_POS_Z = -55332

MAX_ALTITUDE = 200
DIST_X = 0 # -100, -70, 0, 70, 100
DIST_Z = 0 # -100, -70, 0, 70, 100

GPS_FAIL_SEC = 5

# PID stage 1
# pitch
Pp1 = 0.04
Ip1 = 0.01
Dp1 = 0.004
# roll
Pr1 = 0.02
Ir1 = 0.0001
Dr1 = 0.002

# PID stage 2
# pitch
Pp2 = 2
Ip2 = 0.002
Dp2 = 1
Ke2 = 0.4
# roll
Pr2 = 2
Ir2 = 0.001
Dr2 = 0.6
Ka2 = 0.4
# yaw
Py2 = 0.8
Iy2 = 0.001
Dy2 = 0.2
Kr2 = 1
# throttle
Pv2 = 0.1
Dv2 = 0.1 
Kv2 = 0.12

# PID stage 3
# pitch
Pp3 = 2
Ip3 = 0.002
Dp3 = 1
Ke3 = 0.4
# roll
Pr3 = 2
Ir3 = 0.001
Dr3 = 0.6
Ka3 = 0.4
# yaw
Py3 = 0.8
Iy3 = 0.001
Dy3 = 0.2
Kr3 = 1
# throttle
Pv3 = 0.2
Dv3 = 0.6
Kv3 = 0.12