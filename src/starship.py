# -*- coding: utf-8 -*-
"""
Created on Sun Jun  2 17:40:13 2024

@author: Ammar Abdurrauf
"""

import xpc
import numpy as np

# coordinate of the landing area
LANDING_CENTER_X = -15531.20
LANDING_CENTER_Z = -55350.81

class Starship():
    def __init__(self):
        self.client = xpc.XPlaneConnect()
        # Verify connection
        try:
            # If X-Plane does not respond to the request, 
            # a timeout error will be raised.
            self.client.getDREF("sim/test/test_float")
        except:
            print("Error establishing connection to X-Plane.")
            self.client.close()
            self.client = None
            return
        
        print("Connection established-xplane")
        
    def set_altitude(self, alt):
        X = -998  # set value to -998 to keep unchanged
        values = [X, X, alt, X, X, X, X]
        try:
            self.client.sendPOSI(values, 0)
        except:
            print("Error setting the altitude.")
    
    def set_position(self, latitude=-998, longitude=-998, altitude=-998, 
                     pitch=-998, roll=-998, true_head=-998, gear=-998):
        
        values = [latitude, longitude, altitude, pitch, roll, true_head, gear]
        try:
            self.client.sendPOSI(values, 0)
        except:
            print("Error setting the position.")
    
    def has_crashed(self):
        try:
            self.client.clearBuffer()
            crash = self.client.getDREF('sim/flightmodel2/misc/has_crashed')[0]
        except:
            crash = 0
            self.client.clearBuffer()
        
        return crash
    
    def is_on_ground(self):
        try:
            self.client.clearBuffer()
            on_ground = self.client.getDREF('sim/flightmodel/failures/onground_any')[0]
        except:
            on_ground = 0
            self.client.clearBuffer()
            
        return on_ground
    
    def crashed_or_landed(self):
        drefs = ['sim/flightmodel2/misc/has_crashed',
                 'sim/flightmodel/failures/onground_any']
        try:
            self.client.clearBuffer()
            values = self.client.getDREFs(drefs)
            values = np.array(values).flatten()
        except:
            self.client.clearBuffer()
            # set values to 0 if error occurred in communication with x-plane
            values = np.zeros(2, dtype=int)
            
        return values
    
    def send_control(self, control_values = [], 
                     elevator=-998, aileron=-998, rudder=-998, 
                     throttle=-998, gear=-998):
        
        if len(control_values) == 0:
            controls = [elevator, aileron, rudder, throttle, gear, -998]
        else:
            controls = control_values            
            
        try:
            self.client.sendCTRL(controls)
        except:
            pass
        
    def open_landing_legs(self):
        self.send_control(gear=1)
    
    def get_states(self):
        
        """
        Generally:

            sim/cockpit2 are in-cockpit indications - they will show the wrong 
            values when there is a systems failure. For example, the airspeed 
            in sim/cockpit2 will go to zero as the pitot tube ices up.

            sim/flightmodel2 are the real physics-simulation values - 
            they are correct always, and thus are good for modeling the 
            external plane. For example, the AoA in sim/flightmodel2 will 
            always show the real AoA no matter what electrical and vacuum 
            systems fail - this would be appropriate for the AoA sensor 
            mechanism on the outside of the plane.
            
            both sim/cockpit2 and sim/flightmodel2 work correctly for any 
            plane (the user’s or multi-player). This means the sim/flightmodel2 
            datarefs will show the values for the AI/multiplayer planes when 
            read from an object attached to an ACF loaded as an 
            AI/multiplayer plane.

            The older sim/cockpit and sim/flightmodel datarefs are not 
            divided into cockpit (with failures) vs. physics (no failures), 
            and always show the user’s plane, even when your OBJ is attached 
            to another plane. So it is best to use sim/cockpit2 for generic 
            instruments and sim/ flightmodel2/ everywhere else.
            
            https://developer.x-plane.com/manuals/planemaker/index.html#workingwiththeaircraftssystems
        """
        
        drefs = ['sim/flightmodel2/position/y_agl', # altitude
                 'sim/flightmodel/position/local_x', # position x
                 'sim/flightmodel/position/local_z', # position z (position y is the altitude)
                 'sim/flightmodel/position/vh_ind', # vertical velocity
                 'sim/flightmodel/position/theta', # pitch angle
                 'sim/flightmodel/position/phi', # roll angle
                 
                 'sim/flightmodel/position/psi', # yaw angle
                 # 'sim/flightmodel/forces/vx_acf_axis', # velocity across x axis
                 # 'sim/flightmodel/forces/vz_acf_axis', # velocity acros z axis
                 'sim/flightmodel/position/local_vx', # velocity across x axis
                 'sim/flightmodel/position/local_vz', # velocity acros z axis
                 'sim/flightmodel/position/Q', # pitch rate
                 'sim/flightmodel/position/P', # roll rate
                 'sim/flightmodel/position/R', # yaw rate
                 'sim/flightmodel2/misc/has_crashed'] # crash or not
        try:
            self.client.clearBuffer()
            values = self.client.getDREFs(drefs)
            values = np.array(values).flatten()            
            
            if len(values) < 13:
                raise Exception()

            values[0] = self.ft_to_meter(values[0]) # convert feet to meter
            values[1] = values[1] - LANDING_CENTER_X # normalize the coordinate x and z
            values[2] = values[2] - LANDING_CENTER_Z
        except:
            self.client.clearBuffer()
            # set values to 0 if error occurred in communication with x-plane
            values = np.zeros(13)
            
        return values
    
    def set_pitch_rate(self, angle):
        try:
            self.client.sendDREF('sim/flightmodel/position/Q', angle)
        except:
            pass
        
    def set_roll_rate(self, angle):
        try:
            self.client.sendDREF('sim/flightmodel/position/P', angle)
        except:
            pass
        
    def wait_starship_to_reset(self):
        self.client.clearBuffer()
        crash = self.has_crashed()
        while crash == 1.0:
            self.client.clearBuffer()
            crash = self.has_crashed()
    
    def wait_starship_to_touch_ground(self):
        self.client.clearBuffer()
        on_ground = self.is_on_ground()
        while on_ground == 0.0:
            self.client.clearBuffer()
            on_ground = self.is_on_ground()
    
    def refuel_starship(self, fuel=294835.13):
        try:
            self.client.sendDREF('sim/flightmodel/weight/m_fuel1', fuel)
        except:
            pass
    
    def get_fuel_cost(self, initial_fuel=294835.13):
        try:
            fuel_cost = (initial_fuel-self.client.getDREF('sim/flightmodel/weight/m_fuel1')[0])/initial_fuel
        except:
            pass
        
        return fuel_cost
        
    def sendDREF(self, dref, value):
        try:
            self.client.sendDREF(dref, value)
        except Exception as ex:
            print('error:', ex)
            
    def set_view_spot(self):
        # self.client.sendVIEW(xpc.ViewType.Spot)
        self.client.sendVIEW(xpc.ViewType.Follow)
    
    def pause_sim(self):
        self.client.pauseSim(True)
    
    def resume_sim(self):
        self.client.pauseSim(False)
    
    def ft_to_meter(self, ft):
        return ft*0.3048
        
# st = Starship()
# st.set_altitude(1000)

# import time
# # time.sleep(4)
# crash = st.has_crashed()
# alti = []
# while crash == 0.0:
    
#     state = st.get_states()
#     print("ver velocity:", state[3][0])
#     alti.append(state[0][0])
    
#     # st.pause_sim()
#     # time.sleep(0.05)
#     # st.resume_sim()
    
#     crash = st.has_crashed()
    
# print(len(alti))


