# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 09:07:52 2024

@author: ammar
"""

from utils.utils import switch_tab
from src.starship import Starship
from math import sqrt, e, pi, cos, sin
import pyautogui
import time

def meter_to_ft(m):
    return m/0.3048

# constant
X_DIFF = -170
Z_DIFF = -50

starship = Starship()
switch_tab()
time.sleep(0.2)
pyautogui.keyDown('shift')
pyautogui.press('4')
pyautogui.keyUp('shift')


# set the starship orientation
starship.sendDREF('sim/flightmodel/position/q', [0.8, 0, 0, 0.82])

# change view to main engine
# starship.sendDREF('sim/graphics/view/view_type', 1025)

i = 0
time.sleep(0.2)
# with pyautogui.hold('shift'):
while i < 20:
    i += 1
    pyautogui.keyDown(',')
pyautogui.keyUp(',')

i = 0
while i < 5:
    i += 1
    pyautogui.keyDown('up')
pyautogui.keyUp('up')

# refuel
starship.refuel_starship()

target_altitude = 200
starship.sendDREF('sim/flightmodel/position/local_z', 
                  starship.client.getDREF('sim/flightmodel/position/local_z')[0] + Z_DIFF)
starship.sendDREF('sim/flightmodel/position/local_x', 
                  starship.client.getDREF('sim/flightmodel/position/local_x')[0] + X_DIFF)
starship.set_position(altitude=meter_to_ft(target_altitude))

# pitch
Pp = 2
Ip = 0.002
Dp = 1
Ke = 0.4
# roll
Pr = 2
Ir = 0.001
Dr = 0.6
Ka = 0.4
# yaw
Py = 0.8
Iy = 0.001
Dy = 0.2
Kr = 1
# throttle
Pv = 0.1
Dv = 0.1 
Kv = 0.12
def set_heading(desired_heading):
    q = [0]*4

    starship.client.clearBuffer()
    psi_ = starship.client.getDREF('sim/flightmodel/position/psi')[0]
    theta_ = starship.client.getDREF('sim/flightmodel/position/theta')[0]
    phi_ = starship.client.getDREF('sim/flightmodel/position/phi')[0]
    print(starship.client.getDREF('sim/flightmodel/position/phi')[0])
    print(f'psi: {psi_}\ntheta; {theta_}\nphi: {phi_}\n')
    starship.client.clearBuffer()
        
    psi = pi / 360 * desired_heading
    theta = pi / 360 * theta_
    phi = pi / 360 * phi_
    q[0] =  cos(psi) * cos(theta) * cos(phi) + sin(psi) * sin(theta) * sin(phi)
    q[1] =  cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta) * cos(phi)
    q[2] =  cos(psi) * sin(theta) * cos(phi) + sin(psi) * cos(theta) * sin(phi)
    q[3] = -cos(psi) * sin(theta) * sin(phi) + sin(psi) * cos(theta) * cos(phi)

    starship.sendDREF('sim/flightmodel/position/q', q)

def tanh(x, a=1, b=1):
    return a * ((e**(b*x))-(e**(-b*x)))/((e**(b*x))+(e**(-b*x)))

def hover_with_anti_yaw(target_altitude, altitude, ver_vel, Pv, Dv, 
                         Kv, Ke, Ka, Kr,
                         dx, vel_x, dz, vel_z,
                         pitch, pitch_rate, pitch_integral,
                         roll, roll_rate, roll_integral,
                         yaw, yaw_rate, yaw_integral,
                         Pp, Ip, Dp, 
                         Pr, Ir, Dr,
                         Py, Iy, Dy):
    
    # -----------------------------------------------------
    distance_to_landing_pad = sqrt((dx**2) + (dz**2))
    altitude_error = target_altitude - altitude + 0.8 * distance_to_landing_pad
    
    horizontal_vel_mag = sqrt((vel_x**2) + (vel_z**2))
    ver_vel_error = -ver_vel + 0.4 * horizontal_vel_mag
    
    main_engine = (altitude_error * Pv + ver_vel_error * Dv) * Kv
    # -----------------------------------------------------
    pitch_ref = 0
    pitch_error = pitch_ref - 0.1 * pitch + 0.1 * dx
    pitch_dterror = - 1.2 * pitch_rate + 0.1 * vel_x
    elevator_side_thruster = tanh(pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp, a=1, b=0.12)
    # -----------------------------------------------------
    roll_ref = 0
    roll_error = roll_ref - 0.1 * roll - 0.1 * dz
    roll_dterror = - 1.2 * roll_rate - 0.1 * vel_z
    aileron_side_thruster = tanh(roll_error * Pr + roll_integral * Ir + roll_dterror * Dr, a=1, b=0.12)
    # -----------------------------------------------------  
    pitch_ref = 0
    pitch_error = pitch_ref - 1 * pitch
    pitch_dterror = - 1.2 * pitch_rate
    # if (abs(dx) > 3 and altitude < target_altitude):
    if (abs(dx) > 1.5):
        pitch_error = tanh(pitch_error - 0.6 * dx, a=50, b=0.03)
        pitch_dterror = pitch_dterror + 0.6 * vel_x
    elevator = tanh((pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp) * Ke, a=1, b=0.12)
    # -----------------------------------------------------
    roll_ref = 0
    roll_error = roll_ref - 1 * roll
    roll_dterror = - 1.2 * roll_rate
    if (abs(dz) > 1.5):
        roll_error = tanh(roll_error + 0.6 * dz, a=50, b=0.03)
        roll_dterror = roll_dterror - 0.6 * vel_z
    aileron = tanh((roll_error * Pr + roll_integral * Ir + roll_dterror * Dr) * Ka, a=1, b=0.12)
    # -----------------------------------------------------
    yaw_ref = 90
    yaw_error = yaw_ref - 1 * yaw
    yaw_dterror = -yaw_rate
    rudder = tanh((yaw_error * Py + yaw_integral * Iy + yaw_dterror * Dy) * Kr, a=1.0, b=0.12)
    # -----------------------------------------------------
    
    pitch_error = pitch_ref - pitch
    roll_error = roll_ref - roll
    
    thruster_3 = abs(elevator_side_thruster) if elevator_side_thruster < 0 else 0
    thruster_4 = elevator_side_thruster if elevator_side_thruster > 0 else 0
    thruster_5 = aileron_side_thruster if aileron_side_thruster > 0 else 0
    thruster_6 = abs(aileron_side_thruster) if aileron_side_thruster < 0 else 0
    
    main_engines = max(min(main_engine, 1), 0)
    
    print(f'altitude: {altitude}\ndx: {dx}\ndz: {dz}\n')
    
    
    engine_values = [main_engines, main_engines, main_engines, 
                     thruster_3, thruster_4, thruster_5, thruster_6, 0]
    print(f'engine_values: {engine_values}\n')
    
    return engine_values, elevator, aileron, rudder


pitch_integral = 0
roll_integral = 0
yaw_integral = 0
integral_limit = 3

yaw_ref = 90

starship.sendDREF('sim/operation/override/override_throttles', 1)
starship.sendDREF('sim/operation/override/override_flightcontrol', 1)

stage = 1

while True:
    
    if stage == 1:
        values = starship.get_states()
        # extract values
        altitude = values[0]
        pitch = values[4] # pitch error (desired pitch = 0)
        pitch_rate = values[9] # pitch rate error (desired pitch rate = 0)
        roll = values[5] # roll error (desired roll = 0)
        roll_rate = values[10] # roll rate error (desired roll rate = 0)
        yaw = values[6]
        yaw_rate = values[11]
        ver_vel = values[3] # vertical rate error (used when landing)
        dx = values[1] # position along x axis
        dz = values[2] # position along z axis
        vel_x = values[7] # velocity along x axis
        vel_z = values[8] # velocity along z axis
        
        # accumulate error
        pitch_integral += pitch
        roll_integral += roll
        yaw_integral += (yaw_ref - yaw)
    
        # saturate integral
        pitch_integral = max(min(pitch_integral, integral_limit), -integral_limit)
        roll_integral = max(min(roll_integral, integral_limit), -integral_limit)
        
        throttle_values, elevator, aileron, rudder = hover_with_anti_yaw(target_altitude, altitude, ver_vel, Pv, Dv,
                                                                        Kv, Ke, Ka, Kr, 
                                                                        dx, vel_x, dz, vel_z,
                                                                        pitch, pitch_rate, pitch_integral,
                                                                        roll, roll_rate, roll_integral,
                                                                        yaw, yaw_rate, yaw_integral,
                                                                        Pp, Ip, Dp, 
                                                                        Pr, Ir, Dr,
                                                                        Py, Iy, Dy)
        
        starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                    "sim/flightmodel2/controls/pitch_ratio",
                                    "sim/flightmodel2/controls/roll_ratio",
                                    "sim/flightmodel2/controls/heading_ratio"],
                                  [throttle_values, elevator, aileron, rudder])
        
        switch_to_landing_with_yolo = abs(vel_x) + abs(vel_z) + abs(pitch)*0.5 + \
            abs(roll)*0.5 + abs(pitch_rate) + abs(roll_rate) + abs(ver_vel) + abs(dx) + abs(dz)
            
        print(f'switch_to_landing_with_yolo: {switch_to_landing_with_yolo}\n')    
        if switch_to_landing_with_yolo <= 4:
            stage = 2
            target_altitude = 5
    
    elif stage == 2:
        values = starship.get_states()
        # extract values
        altitude = values[0]
        pitch = values[4] # pitch error (desired pitch = 0)
        pitch_rate = values[9] # pitch rate error (desired pitch rate = 0)
        roll = values[5] # roll error (desired roll = 0)
        roll_rate = values[10] # roll rate error (desired roll rate = 0)
        yaw = values[6]
        yaw_rate = values[11]
        ver_vel = values[3] # vertical rate error (used when landing)
        dx = values[1] # position along x axis
        dz = values[2] # position along z axis
        vel_x = values[7] # velocity along x axis
        vel_z = values[8] # velocity along z axis
        
        # accumulate error
        pitch_integral += pitch
        roll_integral += roll
        yaw_integral += (yaw_ref - yaw)
    
        # saturate integral
        pitch_integral = max(min(pitch_integral, integral_limit), -integral_limit)
        roll_integral = max(min(roll_integral, integral_limit), -integral_limit)
        
        Pv = 0.2
        Dv = 0.6
        throttle_values, elevator, aileron, rudder = hover_with_anti_yaw(target_altitude, altitude, ver_vel, Pv, Dv,
                                                                        Kv, Ke, Ka, Kr, 
                                                                        dx, vel_x, dz, vel_z,
                                                                        pitch, pitch_rate, pitch_integral,
                                                                        roll, roll_rate, roll_integral,
                                                                        yaw, yaw_rate, yaw_integral,
                                                                        Pp, Ip, Dp, 
                                                                        Pr, Ir, Dr,
                                                                        Py, Iy, Dy)
        
        starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                    "sim/flightmodel2/controls/pitch_ratio",
                                    "sim/flightmodel2/controls/roll_ratio",
                                    "sim/flightmodel2/controls/heading_ratio"],
                                  [throttle_values, elevator, aileron, rudder])
        
        # target_altitude -= 2
        # target_altitude = target_altitude if target_altitude > 5 else 5
        print(f'ver_vel: {ver_vel}\ntarget_altitude: {target_altitude}\n')
        
    
    if starship.has_crashed() or starship.is_on_ground():
        break

starship.sendDREF('sim/operation/override/override_throttles', 0)
starship.sendDREF('sim/operation/override/override_flightcontrol', 0)