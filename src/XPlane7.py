# -*- coding: utf-8 -*-
"""
Created on Fri Aug 23 21:30:37 2024

@author: Ammar Abdurrauf
version: 7.0
update:
    add set heading, using euler to quaternion (not necessary, we can use starship.set_position(true_head=(#angle in degree)))
    hover to the landing pad using GPS
"""

from ultralytics.utils.plotting import Annotator
from utils.utils import switch_tab
from src.starship import Starship
from ultralytics import YOLO
from wincam import DXCamera
from math import e, sqrt, pi, cos, sin
import numpy as np
import threading
# import pyautogui
import time
import cv2

# constant
CENTER_X = 639
CENTER_Y = 222
# coordinate of the landing area (sometimes change)
LANDING_CENTER_X = -15531.15 # -15526.20
LANDING_CENTER_Z = -55350.81 # 55967.3125

INITIAL_POS_X = -15728
INITIAL_POS_Z = -55332

MAX_ALTITUDE = 200
DIST_X = 100
DIST_Z = -70

class XPlane:
    
    def __init__(self,
                 yolo_model_path="D:\\Projects\\YOLOv8_training\\runs\\detect\\train24\\weights\\best.pt",
                 xplane_screen = [2, 103, 1278, 714],
                 fps=30
                 ):
        self.yolo_model_path = yolo_model_path
        self.yolo_model = YOLO(self.yolo_model_path)
        self.xplane_screen = xplane_screen
        self.fps = fps
        
        self.starship = Starship()
        
        # threads
        self.object_detection_thread = threading.Thread(target=self.object_detection_function, args=())
        self.xplane_communication_thread = threading.Thread(target=self.xplane_communication, args=())
        
        # global variable
        self.run = True
        self.altitude = '##'
        self.target_center = None
        self.prev_target_center = None

    def set_env(self):
        switch_tab()
        time.sleep(0.5)
        
        # disable flightcontrol override first
        self.starship.sendDREF('sim/operation/override/override_flightcontrol', 0)
        
        # set the camera view
        self.starship.sendDREF('sim/graphics/view/view_type', 1025)
        
        # refuel the starship
        self.starship.refuel_starship()
        # set the starship orientation
        self.starship.set_position(true_head=90)
        
        # reposition the rocket
        self.starship.sendDREF('sim/flightmodel/position/local_x', LANDING_CENTER_X+DIST_X)
        self.starship.sendDREF('sim/flightmodel/position/local_z', LANDING_CENTER_Z+DIST_Z)
        
        #######################################################################
        # for testing (setting the altitude of the starship. can be done using self.starship.set_position(altitude=##))
        # target_altitude = self.starship.client.getDREF('sim/flightmodel/position/local_y')[0] + 500
        # self.starship.sendDREF('sim/flightmodel/position/local_y', target_altitude)
        
    def start_sim(self):
        self.set_env()
        time.sleep(1)
        # self.object_detection_thread.start()
        time.sleep(0.5)
        
        # enable throttle override
        self.starship.sendDREF('sim/operation/override/override_throttles', 1)
        
        # throttle on        
        throttle_values = [1.5, 1.5, 1.5, 0, 0, 0, 0, 0]
        self.starship.sendDREF("sim/flightmodel/engine/ENGN_thro_use", throttle_values)
        self.xplane_communication_thread.start()
        
    def wait_threads(self):
        # self.object_detection_thread.join()
        self.xplane_communication_thread.join()
        
        # disable throttle override
        self.starship.sendDREF('sim/operation/override/override_throttles', 0)
        self.starship.sendDREF('sim/operation/override/override_flightcontrol', 0)
        
    def draw_crosshair(self, image):
        color = (100, 100, 100)  # Line color in BGR, e.g., blue
        thickness = 2  # Line thickness in pixels

        # draw the x-y coordinate
        y1_axis_start = (CENTER_X, 0)  # Starting coordinate, e.g., (x1, y1)
        y1_axis_end = (CENTER_X, CENTER_Y-100)  # Ending coordinate, e.g., (x2, y2)
        
        y2_axis_start = (CENTER_X, CENTER_Y+100)  # Starting coordinate, e.g., (x1, y1)
        y2_axis_end = (CENTER_X, 714)  # Ending coordinate, e.g., (x2, y2)
        
        x1_axis_start = (0, CENTER_Y)  # Starting coordinate, e.g., (x1, y1)
        x1_axis_end = (CENTER_X-100, CENTER_Y)  # Ending coordinate, e.g., (x2, y2)
        
        x2_axis_start = (CENTER_X+100, CENTER_Y)  # Starting coordinate, e.g., (x1, y1)
        x2_axis_end = (1278, CENTER_Y)  # Ending coordinate, e.g., (x2, y2)
        
        image = cv2.line(image, y1_axis_start, y1_axis_end, color, thickness)
        image = cv2.line(image, y2_axis_start, y2_axis_end, color, thickness)
        
        image = cv2.line(image, x1_axis_start, x1_axis_end, color, thickness)
        image = cv2.line(image, x2_axis_start, x2_axis_end, color, thickness)
        
        # draw the rectangle
        rect11_start = (CENTER_X-100, CENTER_Y-100)  # Starting coordinate, e.g., (x1, y1)
        rect11_end = (CENTER_X-70, CENTER_Y-100)  # Ending coordinate, e.g., (x2, y2)
        rect12_start = (CENTER_X-100, CENTER_Y-100)  # Starting coordinate, e.g., (x1, y1)
        rect12_end = (CENTER_X-100, CENTER_Y-70)  # Ending coordinate, e.g., (x2, y2)
        
        rect21_start = (CENTER_X+70, CENTER_Y-100)  # Starting coordinate, e.g., (x1, y1)
        rect21_end = (CENTER_X+100, CENTER_Y-100)  # Ending coordinate, e.g., (x2, y2)
        rect22_start = (CENTER_X+100, CENTER_Y-100)  # Starting coordinate, e.g., (x1, y1)
        rect22_end = (CENTER_X+100, 287)  # Ending coordinate, e.g., (x2, y2)
        
        rect31_start = (CENTER_X-100, CENTER_Y+100)  # Starting coordinate, e.g., (x1, y1)
        rect31_end = (CENTER_X-70, CENTER_Y+100)  # Ending coordinate, e.g., (x2, y2)
        rect32_start = (CENTER_X-100, CENTER_Y+70)  # Starting coordinate, e.g., (x1, y1)
        rect32_end = (CENTER_X-100, CENTER_Y+100)  # Ending coordinate, e.g., (x2, y2)
        
        rect41_start = (CENTER_X+70, CENTER_Y+100)  # Starting coordinate, e.g., (x1, y1)
        rect41_end = (CENTER_X+100, CENTER_Y+100)  # Ending coordinate, e.g., (x2, y2)
        rect42_start = (CENTER_X+100, CENTER_Y+70)  # Starting coordinate, e.g., (x1, y1)
        rect42_end = (CENTER_X+100, CENTER_Y+100)  # Ending coordinate, e.g., (x2, y2)
        
        # cross on center
        cross1_start = (CENTER_X, CENTER_Y-15)
        cross1_end = (CENTER_X, CENTER_Y+15)
        
        cross2_start = (CENTER_X-15, CENTER_Y)
        cross2_end = (CENTER_X+15, CENTER_Y)
        
        image = cv2.line(image, rect11_start, rect11_end, color, thickness)
        image = cv2.line(image, rect12_start, rect12_end, color, thickness)
        
        image = cv2.line(image, rect21_start, rect21_end, color, thickness)
        image = cv2.line(image, rect22_start, rect22_end, color, thickness)
        
        image = cv2.line(image, rect31_start, rect31_end, color, thickness)
        image = cv2.line(image, rect32_start, rect32_end, color, thickness)
        
        image = cv2.line(image, rect41_start, rect41_end, color, thickness)
        image = cv2.line(image, rect42_start, rect42_end, color, thickness)
        
        image = cv2.line(image, cross1_start, cross1_end, color, thickness)
        image = cv2.line(image, cross2_start, cross2_end, color, thickness)
        
        # add circle
        image = cv2.circle(image, (CENTER_X, CENTER_Y), 200, color, thickness)
        # image = cv2.circle(image, (CENTER_X, CENTER_Y), 100, color, thickness)
        
        return image
    
    def draw_distance(self, image, target_pos=None):
        
        if target_pos is not None:
            color = (255, 0, 0)  # Line color in BGR, e.g., blue
            thickness = 2  # Line thickness in pixels
            
            line_start = (CENTER_X, CENTER_Y)
            line_end = (int(target_pos[0]), int(target_pos[1]))
            
            image = cv2.line(image, line_start, line_end, color, thickness)
            
            # roll
            # roll_start = (int(target_pos[0]), CENTER_Y-15)
            # roll_end = (int(target_pos[0]), CENTER_Y+15)
            # image = cv2.line(image, roll_start, roll_end, color, thickness)
            
            roll_pts = np.array([
                [int(target_pos[0]), CENTER_Y],
                [int(target_pos[0])-10, CENTER_Y+20],
                [int(target_pos[0])+10, CENTER_Y+20]
                ])
            
            cv2.drawContours(image, [roll_pts], 0, color, -1)
            
            dx_end = (int(target_pos[0]), CENTER_Y)
            image = cv2.line(image, (CENTER_X, CENTER_Y), dx_end, color, thickness)
            
            x = (target_pos[0] - CENTER_X)
            image = cv2.putText(image, str(round(x, 2))+' px', (int(target_pos[0])-35, CENTER_Y-17), 
                                cv2.FONT_HERSHEY_PLAIN, fontScale=2, color=color, thickness=2)
            
            # pitch
            # pitch_start = (CENTER_X-15, int(target_pos[1]))
            # pitch_end = (CENTER_X+15, int(target_pos[1]))
            # image = cv2.line(image, pitch_start, pitch_end, color, thickness)
            
            pitch_pts = np.array([
                [CENTER_X, int(target_pos[1])],
                [CENTER_X-20, int(target_pos[1])-10],
                [CENTER_X-20, int(target_pos[1])+10]
                ])
            
            cv2.drawContours(image, [pitch_pts], 0, color, -1)
            
            dy_end = (CENTER_X, int(target_pos[1]))
            image = cv2.line(image, (CENTER_X, CENTER_Y), dy_end, color, thickness)
            
            dy_end = (CENTER_X, int(target_pos[1]))
            image = cv2.line(image, (CENTER_X, CENTER_Y), dy_end, color, thickness)
            
            y = -(target_pos[1] - CENTER_Y)
            image = cv2.putText(image, str(round(y, 2))+' px', (CENTER_X+20, int(target_pos[1])+10), 
                                cv2.FONT_HERSHEY_PLAIN, fontScale=2, color=color, thickness=2)
            
        return image
            
    def object_detection_function(self):
        loop_time = time.time()
        with DXCamera(self.xplane_screen[0],
                      self.xplane_screen[1],
                      self.xplane_screen[2],
                      self.xplane_screen[3], 
                      capture_cursor=False,
                      fps=self.fps) as camera:
            
            while True:
                
                frame, timestamp = camera.get_bgr_frame()
                frame = np.ascontiguousarray(frame)
                
                results = self.yolo_model(frame, verbose=False)
                
                annotator = Annotator(frame)
                boxes = results[0].boxes
                
                if len(boxes) > 0:
                    index = 0
                    max_conf = boxes[index].conf.item()
                    for i in range(1, len(boxes)):
                        temp_conf = boxes[i].conf.item()
                        if temp_conf > max_conf:
                            index = i
                            max_conf = temp_conf
                    
                    b = boxes[index].xyxy[0]
                    # max_conf *= 100
                    annotator.box_label(b, "dz: {:.2f} m".format(self.altitude), color=(0, 0, 255))
                    
                    self.target_center = boxes[index].xywh[0].tolist()[:2]
                    
                    # keep the previous target center coordinate
                    self.prev_target_center = self.target_center
                
                else:
                    self.target_center = None
                    
                    if self.prev_target_center == None:
                        self.prev_target_center = (CENTER_X, CENTER_Y)
                    
                screenshot = annotator.result()
                
                image = self.draw_crosshair(screenshot)
                image = self.draw_distance(image, self.target_center)
                
                display_screenshot = cv2.resize(image, (639, 357))
                
                cv2.imshow('Starship Landing', display_screenshot)
                
                # print('FPS {}'.format(1 / (time.time() - loop_time)))
                loop_time = time.time()
                
                if cv2.waitKey(1) == ord('q'):
                    cv2.destroyAllWindows()
                    self.run = False
                    break
    
    def set_heading(self, desired_heading=90):
        """
        this method us unnecessary because there is already
        a function from Starship called set_position that
        can set the true heading of the starship:
            starship.set_position(true_head=90)
        """
        q = [0]*4

        self.starship.client.clearBuffer()
        # psi_ = self.starship.client.getDREF('sim/flightmodel/position/psi')[0]
        theta_ = self.starship.client.getDREF('sim/flightmodel/position/theta')[0]
        phi_ = self.starship.client.getDREF('sim/flightmodel/position/phi')[0]
        print(self.starship.client.getDREF('sim/flightmodel/position/phi')[0])
        # print(f'psi: {psi_}\ntheta; {theta_}\nphi: {phi_}\n')
        self.starship.client.clearBuffer()
            
        psi = pi / 360 * desired_heading
        theta = pi / 360 * theta_
        phi = pi / 360 * phi_
        q[0] =  cos(psi) * cos(theta) * cos(phi) + sin(psi) * sin(theta) * sin(phi)
        q[1] =  cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta) * cos(phi)
        q[2] =  cos(psi) * sin(theta) * cos(phi) + sin(psi) * cos(theta) * sin(phi)
        q[3] = -cos(psi) * sin(theta) * sin(phi) + sin(psi) * cos(theta) * cos(phi)

        self.starship.sendDREF('sim/flightmodel/position/q', q)
    
    def _accumulate_intregral_error(self, error, accumulated_error, pi_limit=3.0):
        accumulated_error = max(min(accumulated_error + error, pi_limit), -pi_limit)
        return accumulated_error
    
    def _sigmoid(self, x, a=1, b=1):
        return a / (1 + e**(-b*x))

    def _tanh(self, x, a=1, b=1):
        return a * ((e**(b*x))-(e**(-b*x)))/((e**(b*x))+(e**(-b*x)))    
    
    def hover_to_landing_pad(self, altitude, ver_vel, 
                             Pv, Dv, Kv, Ke, Ka, Kr,
                             dx, vel_x, dz, vel_z,
                             pitch, pitch_rate, pitch_integral,
                             roll, roll_rate, roll_integral,
                             yaw, yaw_rate, yaw_integral,
                             Pp, Ip, Dp, 
                             Pr, Ir, Dr,
                             Py, Iy, Dy):
        
        target_altitude = 6
        # -----------------------------------------------------
        distance_to_landing_pad = sqrt((dx**2) + (dz**2))
        altitude_error = target_altitude - altitude + (0.2 * distance_to_landing_pad) # ruas mulai manajemen hover
        # 0.1, 0.2, 0.8, 2.0
        
        horizontal_vel_mag = sqrt((vel_x**2) + (vel_z**2))
        ver_vel_error = -ver_vel + 0.1 * horizontal_vel_mag
        
        main_engine = (altitude_error * Pv + ver_vel_error * Dv) * Kv
        # -----------------------------------------------------
        pitch_ref = 0
        pitch_error = pitch_ref - 1 * pitch + 0.2 * dx
        pitch_dterror = -pitch_rate + 0.8 * vel_x
        elevator_side_thruster = self._tanh(pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp, a=1, b=0.12)
        # -----------------------------------------------------
        roll_ref = 0
        roll_error = roll_ref - 1 * roll - 0.2 * dz
        roll_dterror = -roll_rate - 0.8 * vel_z
        aileron_side_thruster = self._tanh(roll_error * Pr + roll_integral * Ir + roll_dterror * Dr, a=1, b=0.12)
        # -----------------------------------------------------  
        pitch_ref = 0
        pitch_error = pitch_ref - 1 * pitch
        pitch_dterror = -pitch_rate
        if (abs(dx) > 3 and altitude < 10):
            pitch_error = pitch_error - 0.06 * dx
            pitch_dterror = pitch_dterror - 0.06 * vel_x
        elevator = (pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp) * Ke
        # elevator = self._tanh((pitch_error * Pp + pitch_integral * Ip + pitch_dterror * Dp) * Ke, a=1, b=0.12)
        # -----------------------------------------------------
        roll_ref = 0
        roll_error = roll_ref - 1 * roll
        roll_dterror = -roll_rate
        if (abs(dz) > 3 and altitude < 10):
            roll_error = roll_error + 0.06 * dz
            roll_dterror = roll_dterror + 0.06 * vel_z
        aileron = (roll_error * Pr + roll_integral * Ir + roll_dterror * Dr) * Ka
        # aileron = self._tanh((roll_error * Pr + roll_integral * Ir + roll_dterror * Dr) * Ka, a=1, b=0.12)
        # -----------------------------------------------------
        yaw_ref = 90
        yaw_error = yaw_ref - 1 * yaw
        yaw_dterror = -yaw_rate
        rudder = self._tanh((yaw_error * Py + yaw_integral * Iy + yaw_dterror * Dy) * Kr, a=1.0, b=0.12)
        # -----------------------------------------------------
        
        thruster_3 = abs(elevator_side_thruster) if elevator_side_thruster < 0 else 0
        thruster_4 = elevator_side_thruster if elevator_side_thruster > 0 else 0
        thruster_5 = aileron_side_thruster if aileron_side_thruster > 0 else 0
        thruster_6 = abs(aileron_side_thruster) if aileron_side_thruster < 0 else 0
        
        main_engines = max(min(main_engine, 3), 0)
        elevator = max(min(elevator, 1), -1)
        aileron = max(min(aileron, 1), -1)
        
        print(f'main_engine: {main_engine}\naltitude: {altitude}\naltitude_error: {altitude_error}\nmain_engine: {main_engine}\n' + 
              f'dx: {dx}\npitch_error: {pitch_error}\nelevator_side_thruster: {elevator_side_thruster}\n' +
              f'\ndz: {dz}\nvel_z: {vel_z}\nroll_rate: {roll_rate}\nroll_error: {roll_error}\naileron_side_thruster: {aileron_side_thruster}\n' + 
              f'\nyaw: {yaw}\nrudder: {rudder}\n')
        
        
        engine_values = [main_engines, main_engines, main_engines, 
                         thruster_3, thruster_4, thruster_5, thruster_6, 0]
        print(f'engine_values: {engine_values}\n')
        
        return engine_values, elevator, aileron, rudder
        
    
    def xplane_communication(self):
        landing = False
        landed = [False * 10]
        landed_index = 0
        hover = False
        
        self.pitch_integral = 0
        self.roll_integral = 0
        # pitch
        Pp = 0.04
        Ip = 0.01
        Dp = 0.004
        # roll
        Pr = 0.02
        Ir = 0.0001
        Dr = 0.002
        
        throttle_values = [0] * 8
        
        integral_limit = 3

        self.starship.resume_sim()

        while self.run:
            values = self.starship.get_states()
            self.altitude = values[0]
            # print('altitude:', self.altitude)
            if self.altitude > MAX_ALTITUDE:
                # throttle on        
                throttle_values = [0, 0, 0, 0, 0, 0, 0, 0]
                self.starship.sendDREF("sim/flightmodel/engine/ENGN_thro_use", throttle_values)
            
            if not hover:
                # extract values
                pitch = values[4]
                pitch_rate = values[9]
                roll = values[5]
                roll_rate = values[10]
                ver_vel = values[3]
                
                # accumulate error
                self.pitch_integral += pitch
                self.roll_integral += roll
                
                # saturate integral
                self.pitch_integral = max(min(self.pitch_integral, integral_limit), -integral_limit)
                self.roll_integral = max(min(self.roll_integral, integral_limit), -integral_limit)
                
                elevator = -pitch * Pp + pitch_rate * Dp + self.pitch_integral * Ip
                elevator = max(min(elevator, 1), -1)

                aileron = -roll * Pr + roll_rate * Dr + self.roll_integral * Ir
                aileron = max(min(aileron, 1), -1)
                self.starship.send_control(elevator=elevator, aileron=aileron)
                
                if ver_vel < 0 and self.altitude >=10:
                    hover = True
                    self.pitch_integral = 0
                    self.roll_integral = 0
                    self.yaw_integral = 0
                    
                    self.starship.send_control(elevator=0, aileron=0)
                    
            elif hover:
                # enable throttle override
                self.starship.sendDREF('sim/operation/override/override_flightcontrol', 1)
                # extract values
                pitch = values[4] # pitch error (desired pitch = 0)
                pitch_rate = values[9] # pitch rate error (desired pitch rate = 0)
                roll = values[5] # roll error (desired roll = 0)
                roll_rate = values[10] # roll rate error (desired roll rate = 0)
                yaw = values[6] # yaw
                yaw_rate = values[11] # yaw rate
                ver_vel = values[3] # vertical rate error (used when landing)
                dx = values[1] # position along x axis
                dz = values[2] # position along z axis
                vel_x = values[7] # velocity along x axis
                vel_z = values[8] # velocity along z axis
                
                # accumulate error
                self.pitch_integral += pitch
                self.roll_integral += roll
                self.yaw_integral += (90 - yaw)

                # saturate integral
                self.pitch_integral = max(min(self.pitch_integral, integral_limit), -integral_limit)
                self.roll_integral = max(min(self.roll_integral, integral_limit), -integral_limit)
                self.yaw_integral = max(min(self.yaw_integral, integral_limit), -integral_limit)
                
                
                # pitch
                Pp = 4.0
                Ip = 0.001
                Dp = 2.22
                Ke = 0.1
                # roll
                Pr = 4.0
                Ir = 0.001
                Dr = 2.22
                Ka = 0.1
                # yaw
                Py = 1.8
                Iy = 0.001
                Dy = 0.8
                Kr = 1
                # throttle
                Pv = 0.082
                Dv = 0.112
                Kv = 0.4
                throttle_values, elevator, aileron, rudder = self.hover_to_landing_pad(self.altitude, ver_vel, Pv, Dv,
                                                                                       Kv, Ke, Ka, Kr,
                                                                                       dx, vel_x, dz, vel_z,
                                                                                       pitch, pitch_rate, self.pitch_integral,
                                                                                       roll, roll_rate, self.roll_integral,
                                                                                       yaw, yaw_rate, self.yaw_integral,
                                                                                       Pp, Ip, Dp, 
                                                                                       Pr, Ir, Dr,
                                                                                       Py, Iy, Dy)
                                
                self.starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                                "sim/flightmodel2/controls/pitch_ratio",
                                                "sim/flightmodel2/controls/roll_ratio",
                                                "sim/flightmodel2/controls/heading_ratio"],
                                               [throttle_values, elevator, aileron, rudder])
                
                safe_for_engine_turn_off = abs(self.altitude) + abs(vel_x) + abs(vel_z) + abs(pitch)*0.25 + \
                                            abs(roll)*0.25 + abs(pitch_rate) + abs(roll_rate) + abs(ver_vel)
                
                print(f'safe_for_engine_turn_off: {safe_for_engine_turn_off}')
                
                if (safe_for_engine_turn_off <=2.2):
                    throttle_values = [0,0,0,0,0,0,0,0]
                    self.starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                                    "sim/flightmodel2/controls/pitch_ratio",
                                                    "sim/flightmodel2/controls/roll_ratio"],
                                                   [throttle_values, 0, 0])
                    
                    dfld = sqrt(((dx)**2)+((dz)**2))
                    print(f'distance from landing pad: {dfld}')
                    break
                
                
            if landing and self.starship.is_on_ground():
                landed[landed_index] = True
                landed_index += 1
                
            if False not in landed:
                # self.starship.send_control(throttle=0.0)
                # time.sleep(0.4)
                # self.starship.pause_sim()
                landing = False
                
                
            

sim = XPlane()
sim.start_sim()
time.sleep(1)
sim.wait_threads()


"""
TODO:
    1. The input control should also consider the distance from the taget landing pad
    2. The distance between the rocket to the landing pad also depends on the altitude of the rocket

Method:
    Measure the distance between the center of the camera and the center of the 
    landing pad (bounding box from the detected landing path). Map the altitude
    of the rocket to the distance to the landing pad.

Question:
    1. should the rocket hover first and move to the target landing first then 
    descent to the ground? or should it descent while also moving toward the landing pad?

"""



