# -*- coding: utf-8 -*-
"""
Created on Mon Aug 19 15:00:44 2024

@author: ammar
"""
from utils.utils import switch_tab
from src.starship import Starship
from math import sqrt, e
import pyautogui
import time

starship = Starship()
switch_tab()
time.sleep(0.2)
pyautogui.keyDown('shift')
pyautogui.press('4')
pyautogui.keyUp('shift')

# set the starship orientation
starship.sendDREF('sim/flightmodel/position/q', [0.8, 0, 0, 0.82])

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

starship.refuel_starship()

target_altitude = starship.client.getDREF('sim/flightmodel/position/local_y')[0] + 500
starship.sendDREF('sim/flightmodel/position/local_z', 
                  starship.client.getDREF('sim/flightmodel/position/local_z')[0] + 100)
starship.sendDREF('sim/flightmodel/position/local_x', 
                  starship.client.getDREF('sim/flightmodel/position/local_x')[0] + 100)
starship.sendDREF('sim/flightmodel/position/local_y', target_altitude)

# pitch
Pp = 1.1
Ip = 0.00001
Dp = 0.2
Ke = 0.1
# roll
Pr = 1.1
Ir = 0.00001
Dr = 0.2
Ka = 0.1
# throttle
Pv = 0.1
Dv = 0.1 
Kv = 0.1

def tanh(x, a=1, b=1):
    return a * ((e**(b*x))-(e**(-b*x)))/((e**(b*x))+(e**(-b*x)))

def hover_to_landing_pad(altitude, ver_vel, Pv, Dv, 
                         Kv, Ke, Ka,
                         dx, vel_x, dz, vel_z,
                         pitch, pitch_rate, pitch_integral,
                         roll, roll_rate, roll_integral,
                         Pp, Ip, Dp, 
                         Pr, Ir, Dr):
    
    # -----------------------------------------------------
    distance_to_landing_pad = sqrt((dx**2) + (dz**2))
    altitude_error = target_altitude - altitude + 0.1 * distance_to_landing_pad
    
    horizontal_vel_mag = sqrt((vel_x**2) + (vel_z**2))
    ver_vel_error = -ver_vel + 0.1 * horizontal_vel_mag
    
    main_engine = (altitude_error * Pv + ver_vel_error * Dv) * Kv
    # -----------------------------------------------------
    pitch_ref = 0
    pitch_error = pitch_ref - 1 * pitch + 0.2 * dx
    pitch_dterror = -pitch_rate + 0.8 * vel_x
    elevator_side_thruster = tanh(pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp, a=1, b=0.12)
    # -----------------------------------------------------
    roll_ref = 0
    roll_error = roll_ref - 1 * roll - 0.2 * dz
    roll_dterror = -roll_rate - 0.8 * vel_z
    aileron_side_thruster = tanh(roll_error * Pr + roll_integral * Ir + roll_dterror * Dr, a=1, b=0.12)
    # -----------------------------------------------------  
    pitch_ref = 0
    pitch_error = pitch_ref - 1 * pitch
    pitch_dterror = -pitch_rate
    if (abs(dx) > 3 and altitude < 10):
        pitch_error = pitch_error - 0.06 * dx
        pitch_dterror = pitch_dterror - 0.06 * vel_x
    # elevator = (pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp) * Ke
    elevator = tanh((pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp) * Ke, a=1, b=0.12)
    # -----------------------------------------------------
    roll_ref = 0
    roll_error = roll_ref - 1 * roll
    roll_dterror = -roll_rate
    if (abs(dz) > 3 and altitude < 10):
        roll_error = roll_error + 0.06 * dz
        roll_dterror = roll_dterror + 0.06 * vel_z
    # aileron = (roll_error * Pr + roll_integral * Ir + roll_dterror * Dr) * Ka
    aileron = tanh((roll_error * Pr + roll_integral * Ir + roll_dterror * Dr) * Ka, a=1, b=0.12)
    # -----------------------------------------------------
    
    thruster_3 = abs(elevator_side_thruster) if elevator_side_thruster < 0 else 0
    thruster_4 = elevator_side_thruster if elevator_side_thruster > 0 else 0
    thruster_5 = aileron_side_thruster if aileron_side_thruster > 0 else 0
    thruster_6 = abs(aileron_side_thruster) if aileron_side_thruster < 0 else 0
    
    # thruster_3 = abs(min(max(elevator_side_thruster, -1), 0)) # if pitch_rate < 0 else pitch_rate * Dp)
    # thruster_4 = abs(max(min(elevator_side_thruster, 1), 0)) # if pitch_rate > 0 else pitch_rate * Dp)
    # thruster_5 = abs(max(min(aileron_side_thruster, 1), 0)) # if roll_rate > 0 else roll_rate * Dr)
    # thruster_6 = abs(min(max(aileron_side_thruster, -1), 0)) # if roll_rate < 0 else roll_rate * Dr)
    
    main_engines = max(min(main_engine, 1), 0)
    elevator = max(min(elevator, 1), -1)
    aileron = max(min(aileron, 1), -1)
    
    print(f'main_engine: {main_engine}\naltitude: {altitude}\naltitude_error: {altitude_error}\nmain_engine: {main_engine}\n' + 
          f'dx: {dx}\npitch_error: {pitch_error}\nelevator_side_thruster: {elevator_side_thruster}\n' +
          f'\ndz: {dz}\nvel_z: {vel_z}\nroll_rate: {roll_rate}\nroll_error: {roll_error}\naileron_side_thruster: {aileron_side_thruster}\n')
    
    
    engine_values = [main_engines, main_engines, main_engines, 
                     thruster_3, thruster_4, thruster_5, thruster_6, 0]
    print(f'engine_values: {engine_values}\n')
    
    return engine_values, elevator, aileron


pitch_integral = 0
roll_integral = 0
integral_limit = 3

starship.sendDREF('sim/operation/override/override_throttles', 1)
starship.sendDREF('sim/operation/override/override_flightcontrol', 1)
while True:
    values = starship.get_states()
    # extract values
    altitude = starship.client.getDREF('sim/flightmodel/position/local_y')[0]
    pitch = values[4] # pitch error (desired pitch = 0)
    pitch_rate = values[9] # pitch rate error (desired pitch rate = 0)
    roll = values[5] # roll error (desired roll = 0)
    roll_rate = values[10] # roll rate error (desired roll rate = 0)
    yaw = values[6] # yaw angle
    ver_vel = values[3] # vertical rate error (used when landing)
    dx = values[1] # position along x axis
    dz = values[2] # position along z axis
    vel_x = values[7] # velocity along x axis
    vel_z = values[8] # velocity along z axis
    
    # accumulate error
    pitch_integral += pitch
    roll_integral += roll

    # saturate integral
    pitch_integral = max(min(pitch_integral, integral_limit), -integral_limit)
    roll_integral = max(min(roll_integral, integral_limit), -integral_limit)
    
    throttle_values, elevator, aileron = hover_to_landing_pad(altitude, ver_vel, Pv, Dv,
                                           Kv, Ke, Ka, dx, 
                                           vel_x, dz, vel_z,
                                           pitch, pitch_rate, pitch_integral,
                                           roll, roll_rate, roll_integral,
                                           Pp, Ip, Dp, 
                                           Pr, Ir, Dr)
    
    starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                "sim/flightmodel2/controls/pitch_ratio",
                                "sim/flightmodel2/controls/roll_ratio"],
                              [throttle_values, elevator, aileron])
    
    # starship.sendDREF("sim/flightmodel/engine/ENGN_thro_use", throttle_values)
    # starship.sendDREF("sim/flightmodel2/controls/pitch_ratio", elevator)
    # starship.sendDREF("sim/flightmodel2/controls/roll_ratio", aileron)

    if starship.has_crashed():
        break

starship.sendDREF('sim/operation/override/override_throttles', 0)
starship.sendDREF('sim/operation/override/override_flightcontrol', 0)