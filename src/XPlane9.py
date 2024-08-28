# -*- coding: utf-8 -*-
"""
Created on Sun Aug 25 08:58:45 2024

@author: Ammar Abdurrauf
version: 9.0
update:
    launch - hover - land
"""

from src.constants9 import CENTER_X, CENTER_Y, LANDING_CENTER_X, LANDING_CENTER_Z, GPS_FAIL_SEC
from src.constants9 import INITIAL_POS_X, INITIAL_POS_Z, MAX_ALTITUDE, DIST_X, DIST_Z
from src.constants9 import Pp2, Ip2, Dp2, Ke2, Pr2, Ir2, Dr2, Ka2, Py2, Iy2, Dy2, Kr2, Pv2, Dv2, Kv2
from src.constants9 import Pp3, Ip3, Dp3, Ke3, Pr3, Ir3, Dr3, Ka3, Py3, Iy3, Dy3, Kr3, Pv3, Dv3, Kv3
from src.constants9 import Pp1, Ip1, Dp1, Pr1, Ir1, Dr1, FLIGTH_DATA_DIR, FLIGTH_DATA_FILENAME, CONTROL_DATA_FILENAME
from utils.utils import switch_tab, save_data_to_csv
from ultralytics.utils.plotting import Annotator
from utils.utils import switch_tab
from src.starship import Starship
from ultralytics import YOLO
from wincam import DXCamera
from math import e, sqrt
import numpy as np
import threading
# import pyautogui
import time
import cv2
import os



class XPlane:
    
    def __init__(self,
                 yolo_model_path="D:\\Projects\\xplane_yolo\\yolo\\n\\weights\\best.pt",
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
        self.data_writer_thread = threading.Thread(target=self.data_writer_function, args=())
        self.gps_failure_thread = threading.Thread(target=self.gps_fail, args=())
        
        # global variable
        self.run = True
        self.target_center = None
        self.prev_target_center = None
        self.record_data = True
        self.gps_fail_flag = False
        
        self.altitude = None
        self.ver_vel = None
        self.pitch = None
        self.pitch_rate = None
        self.roll = None
        self.roll_rate = None
        self.yaw = None
        self.yaw_rate = None
        self.ver_vel = None
        self.dx = None
        self.dz = None
        self.vel_x = None
        self.vel_z = None
        self.FPS = None
        self.has_crashed = False
        
        self.main_engine = 0
        self.thruster_3 = 0
        self.thruster_4 = 0
        self.thruster_5 = 0
        self.thruster_6 = 0
        self.elevator = 0
        self.aileron = 0
        self.rudder = 0


        self.data_writer_event = threading.Event()
        
        # data
        self.data = {'altitude': [],
                     'ver vel': [],
                     'pitch': [],
                     'pitch rate': [],
                     'roll': [],
                     'roll rate': [],
                     'yaw': [],
                     'yaw rate': [],
                     'pos x': [],
                     'pos y': [],
                     'vel x': [],
                     'vel y': [],
                     'fps': []}
        
        self.control_data = {'main eng': [],
                             'thurst 3': [],
                             'thurst 4': [],
                             'thurst 5': [],
                             'thurst 6': [],
                             'elevator': [],
                             'aileron': [],
                             'rudder': []}

    def set_env(self):
        switch_tab()
        time.sleep(0.5)
        
        # disable flightcontrol override first
        self.starship.sendDREF('sim/operation/override/override_flightcontrol', 0)
        
        # set the camera view
        # self.starship.sendDREF('sim/graphics/view/view_type', 1025)
        
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
        self.data_writer_thread.start()
        time.sleep(0.5)
        
        # enable throttle override
        self.starship.sendDREF('sim/operation/override/override_throttles', 1)
        
        # throttle on
        self.main_engine = 1.5
        throttle_values = [self.main_engine, self.main_engine, self.main_engine, 0, 0, 0, 0, 0]
        self.starship.sendDREF("sim/flightmodel/engine/ENGN_thro_use", throttle_values)
        self.xplane_communication_thread.start()
        
    def wait_threads(self):
        # self.object_detection_thread.join()
        self.xplane_communication_thread.join()
        self.data_writer_thread.join()
        # self.gps_failure_thread.join()
        
        # disable throttle override
        self.starship.sendDREF('sim/operation/override/override_throttles', 0)
        self.starship.sendDREF('sim/operation/override/override_flightcontrol', 0)
    
    def gps_fail(self):
        while self.run:
            if not self.altitude == None and not self.ver_vel == None:
                if self.altitude < 120 and self.ver_vel < 0:
                    self.gps_fail_flag = True
                    time.sleep(GPS_FAIL_SEC)
                    self.gps_fail_flag = False
                    break
    
    def data_writer_function(self):
        while self.record_data:
            
            self.data_writer_event.wait()
            self.data['altitude'].append(self.altitude)
            self.data['ver vel'].append(self.ver_vel)
            self.data['pitch'].append(self.pitch)
            self.data['pitch rate'].append(self.pitch_rate)
            self.data['roll'].append(self.roll)
            self.data['roll rate'].append(self.roll_rate)
            self.data['yaw'].append(self.yaw)
            self.data['yaw rate'].append(self.yaw_rate)
            self.data['pos x'].append(self.dx)
            self.data['pos y'].append(self.dz)
            self.data['vel x'].append(self.vel_x)
            self.data['vel y'].append(self.vel_z)
            self.data['fps'].append(self.FPS)
            
            self.control_data['main eng'].append(self.main_engine)
            self.control_data['thurst 3'].append(self.thruster_3)
            self.control_data['thurst 4'].append(self.thruster_4)
            self.control_data['thurst 5'].append(self.thruster_5)
            self.control_data['thurst 6'].append(self.thruster_6)
            self.control_data['elevator'].append(self.elevator)
            self.control_data['aileron'].append(self.aileron)
            self.control_data['rudder'].append(self.rudder)
            
            time.sleep(0.1)
    
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
    
    def _accumulate_intregral_error(self, error, accumulated_error, pi_limit=3.0):
        accumulated_error = max(min(accumulated_error + error, pi_limit), -pi_limit)
        return accumulated_error
    
    def _sigmoid(self, x, a=1, b=1):
        return a / (1 + e**(-b*x))

    def _tanh(self, x, a=1, b=1):
        return a * ((e**(b*x))-(e**(-b*x)))/((e**(b*x))+(e**(-b*x)))    
    
    def pid_launch(self):
        
        integral_limit = 3
        
        # accumulate error
        self.pitch_integral += self.pitch
        self.roll_integral += self.roll
        
        # saturate integral
        self.pitch_integral = max(min(self.pitch_integral, integral_limit), -integral_limit)
        self.roll_integral = max(min(self.roll_integral, integral_limit), -integral_limit)
        
        elevator = -self.pitch * Pp1 + self.pitch_rate * Dp1 + self.pitch_integral * Ip1
        self.elevator = max(min(elevator, 1), -1)

        aileron = -self.roll * Pr1 + self.roll_rate * Dr1 + self.roll_integral * Ir1
        self.aileron = max(min(aileron, 1), -1)
        self.starship.send_control(elevator=self.elevator, aileron=self.aileron)
        
    def hover_to_landing_pad(self, target_altitude):
        # -----------------------------------------------------
        distance_to_landing_pad = sqrt((self.dx**2) + (self.dz**2))
        altitude_error = target_altitude - self.altitude + 0.5 * distance_to_landing_pad
        
        horizontal_vel_mag = sqrt((self.vel_x**2) + (self.vel_z**2))
        ver_vel_error = -2*self.ver_vel + 0.4 * horizontal_vel_mag
        
        main_engine = (altitude_error * Pv2 + ver_vel_error * Dv2) * Kv2
        # -----------------------------------------------------
        pitch_ref = 0
        pitch_error =  pitch_ref - 0.14 * self.pitch + 0.1 * self.dx
        pitch_dterror = - 1.2 * self.pitch_rate + 1.1 * self.vel_x
        elevator_side_thruster = self._tanh(pitch_error * Pp2 + self.pitch_integral * Ip2 + pitch_dterror * Dp2, a=1, b=0.12)
        # -----------------------------------------------------
        roll_ref = 0
        roll_error =  roll_ref - 0.14 * self.roll - 0.1 * self.dz
        roll_dterror = - 1.2 * self.roll_rate - 1.1 * self.vel_z
        aileron_side_thruster = self._tanh(roll_error * Pr2 + self.roll_integral * Ir2 + roll_dterror * Dr2, a=1, b=0.12)
        # -----------------------------------------------------  
        pitch_ref = 0
        pitch_error =  self._tanh(pitch_ref - 1 * self.pitch, a=30, b=0.05)
        pitch_dterror = - 1.2 * self.pitch_rate
        # if (abs(dx) > 3 and altitude < target_altitude):
        if (abs(self.dx) > 1.5):
            pitch_error = self._tanh(pitch_error - 0.6 * self.dx, a=30, b=0.05)
            pitch_dterror = pitch_dterror + 0.6 * self.vel_x
        self.elevator = self._tanh((pitch_error * Pp2 + self.pitch_integral * Ip2 + pitch_dterror * Dp2) * Ke2, a=1, b=0.12)
        # -----------------------------------------------------
        roll_ref = 0
        roll_error =  self._tanh(roll_ref - 1 * self.roll, a=30, b=0.05)
        roll_dterror = - 1.2 * self.roll_rate
        if (abs(self.dz) > 1.5):
            roll_error = self._tanh(roll_error + 0.6 * self.dz, a=30, b=0.05)
            roll_dterror = roll_dterror - 0.6 * self.vel_z
        self.aileron = self._tanh((roll_error * Pr2 + self.roll_integral * Ir2 + roll_dterror * Dr2) * Ka2, a=1, b=0.12)
        # -----------------------------------------------------
        yaw_ref = 90
        yaw_error = yaw_ref - 1 * self.yaw
        yaw_dterror = -self.yaw_rate
        self.rudder = self._tanh((yaw_error * Py2 + self.yaw_integral * Iy2 + yaw_dterror * Dy2) * Kr2, a=1.0, b=0.12)
        # -----------------------------------------------------
        
        # pitch_error = pitch_ref - self.pitch
        # roll_error = roll_ref - self.roll
        
        self.thruster_3 = abs(elevator_side_thruster) if elevator_side_thruster < 0 else 0
        self.thruster_4 = elevator_side_thruster if elevator_side_thruster > 0 else 0
        self.thruster_5 = aileron_side_thruster if aileron_side_thruster > 0 else 0
        self.thruster_6 = abs(aileron_side_thruster) if aileron_side_thruster < 0 else 0
        
        self.main_engine = max(min(main_engine, 1), 0)
        
        print(f'altitude: {self.altitude}\ndx: {self.dx}\ndz: {self.dz}\n')
        
        
        engine_values = [self.main_engine, self.main_engine, self.main_engine, 
                         self.thruster_3, self.thruster_4, self.thruster_5, self.thruster_6, 0]
        # print(f'engine_values: {engine_values}\n')
        
        return engine_values
    
    def landing(self, target_altitude):
        
        # -----------------------------------------------------
        distance_to_landing_pad = sqrt((self.dx**2) + (self.dz**2))
        altitude_error = target_altitude - self.altitude + 0.5 * distance_to_landing_pad
        
        horizontal_vel_mag = sqrt((self.vel_x**2) + (self.vel_z**2))
        ver_vel_error = -self.ver_vel + 0.4 * horizontal_vel_mag
        
        main_engine = (altitude_error * Pv3 + ver_vel_error * Dv3) * Kv3
        # -----------------------------------------------------
        pitch_ref = 0
        pitch_error =  pitch_ref - 0.14 * self.pitch + 0.1 * self.dx
        pitch_dterror = - 1.2 * self.pitch_rate + 1.1 * self.vel_x
        elevator_side_thruster = self._tanh(pitch_error * Pp3 + self.pitch_integral * Ip3 + pitch_dterror * Dp3, a=1, b=0.12)
        # -----------------------------------------------------
        roll_ref = 0
        roll_error =  roll_ref - 0.14 * self.roll - 0.1 * self.dz
        roll_dterror = - 1.2 * self.roll_rate - 1.1 * self.vel_z
        aileron_side_thruster = self._tanh(roll_error * Pr3 + self.roll_integral * Ir3 + roll_dterror * Dr3, a=1, b=0.12)
        # -----------------------------------------------------  
        pitch_ref = 0
        pitch_error =  self._tanh(pitch_ref - 1 * self.pitch, a=30, b=0.05)
        pitch_dterror = - 1.2 * self.pitch_rate
        # if (abs(dx) > 3 and altitude < target_altitude):
        if (abs(self.dx) > 1.5):
            pitch_error = self._tanh(pitch_error - 0.6 * self.dx, a=30, b=0.05)
            pitch_dterror = pitch_dterror + 0.6 * self.vel_x
        self.elevator = self._tanh((pitch_error * Pp3 + self.pitch_integral * Ip3 + pitch_dterror * Dp3) * Ke3, a=1, b=0.12)
        # -----------------------------------------------------
        roll_ref = 0
        roll_error =  self._tanh(roll_ref - 1 * self.roll, a=30, b=0.05)
        roll_dterror = - 1.2 * self.roll_rate
        if (abs(self.dz) > 1.5):
            roll_error = self._tanh(roll_error + 0.6 * self.dz, a=30, b=0.05)
            roll_dterror = roll_dterror - 0.6 * self.vel_z
        self.aileron = self._tanh((roll_error * Pr3 + self.roll_integral * Ir3 + roll_dterror * Dr3) * Ka3, a=1, b=0.12)
        # -----------------------------------------------------
        yaw_ref = 90
        yaw_error = yaw_ref - 1 * self.yaw
        yaw_dterror = -self.yaw_rate
        self.rudder = self._tanh((yaw_error * Py3 + self.yaw_integral * Iy3 + yaw_dterror * Dy3) * Kr3, a=1.0, b=0.12)
        # -----------------------------------------------------
        
        self.thruster_3 = abs(elevator_side_thruster) if elevator_side_thruster < 0 else 0
        self.thruster_4 = elevator_side_thruster if elevator_side_thruster > 0 else 0
        self.thruster_5 = aileron_side_thruster if aileron_side_thruster > 0 else 0
        self.thruster_6 = abs(aileron_side_thruster) if aileron_side_thruster < 0 else 0
        
        self.main_engine = max(min(main_engine, 1), 0)
        
        print(f'altitude: {self.altitude}\ndx: {self.dx}\ndz: {self.dz}\n')
        
        
        engine_values = [self.main_engine, self.main_engine, self.main_engine, 
                         self.thruster_3, self.thruster_4, self.thruster_5, self.thruster_6, 0]
        # print(f'engine_values: {engine_values}\n')
        
        return engine_values

    def xplane_communication(self):
        # self.gps_failure_thread.start()
        
        self.pitch_integral = 0
        self.roll_integral = 0
        
        stage = 1
        integral_limit = 3
        yaw_ref = 90
        target_altitude = MAX_ALTITUDE + 70
        behavior = 9999
                
        self.starship.resume_sim()
        loop_time = time.time()
        
        delay_dx_dy = 0
        prev_dx = 0
        prev_dz = 0
        prev_alt = 0
        while self.run:
            values = self.starship.get_states()

            # extract values
            self.altitude = values[0]
            self.dx = values[1] # position along x axis
            self.dz = values[2] # position along z axis
            self.ver_vel = values[3] # vertical rate error (used when landing)
            self.pitch = values[4] # pitch error (desired pitch = 0)
            self.pitch_rate = values[9] # pitch rate error (desired pitch rate = 0)
            self.roll = values[5] # roll error (desired roll = 0)
            self.roll_rate = values[10] # roll rate error (desired roll rate = 0)
            self.yaw = values[6]
            self.yaw_rate = values[11]
            self.vel_x = values[7] # velocity along x axis
            self.vel_z = values[8] # velocity along z axis
            self.has_crashed = values[12]

            if self.altitude < 200 and self.altitude > 20 and self.ver_vel < 0:
                
                if delay_dx_dy % 50 == 0:
                    self.altitude = values[0]
                    self.dx = values[1] # position along x axis
                    self.dz = values[2] # position along z axis
                else:
                    self.altitude = prev_alt
                    self.dx = prev_dx
                    self.dz = prev_dz
                
                print("\n-----------\nGPS FAILURE\n-------------")
            else:
                if delay_dx_dy % 10 == 0:
                    self.altitude = values[0]
                    self.dx = values[1] # position along x axis
                    self.dz = values[2] # position along z axis
                        
                else:
                    self.altitude = prev_alt
                    self.dx = prev_dx
                    self.dz = prev_dz
            
            prev_alt = self.altitude
            prev_dx = self.dx
            prev_dz = self.dz
            
            delay_dx_dy += 1
            
            self.FPS = 1 / (time.time() - loop_time)
            print('FPS {}'.format(self.FPS))
            loop_time = time.time()
            
            self.data_writer_event.set()
            print(f'stage: {stage}')
            # print('altitude:', self.altitude)
            # print('target_altitude', target_altitude)
            
            if self.altitude > MAX_ALTITUDE:
                # throttle on
                self.main_engine = 0
                throttle_values = [0, 0, 0, 0, 0, 0, 0, 0]
                self.starship.sendDREF("sim/flightmodel/engine/ENGN_thro_use", throttle_values)
                
                self.rocket_enginge_off_alt = self.altitude
            
            self.data_writer_event.clear()
            
            if stage == 1:
                self.pid_launch()
                
                if (self.ver_vel < 0 and self.altitude >=10) or self.has_crashed:
                    stage = 2
                    self.pitch_integral = 0
                    self.roll_integral = 0
                    self.yaw_integral = 0
                    self.starship.send_control(elevator=0, aileron=0)
                    self.starship.sendDREF('sim/operation/override/override_flightcontrol', 1)
            
            elif stage == 2:
                
                # accumulate error
                self.pitch_integral += self.pitch
                self.roll_integral += self.roll
                self.yaw_integral += (yaw_ref - self.yaw)

                # saturate integral
                self.pitch_integral = max(min(self.pitch_integral, integral_limit), -integral_limit)
                self.roll_integral = max(min(self.roll_integral, integral_limit), -integral_limit)
                # self.yaw_integral = max(min(self.yaw_integral, integral_limit), -integral_limit)
                
                throttle_values = self.hover_to_landing_pad(target_altitude)
                
                self.starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                            "sim/flightmodel2/controls/pitch_ratio",
                                            "sim/flightmodel2/controls/roll_ratio",
                                            "sim/flightmodel2/controls/heading_ratio"],
                                          [throttle_values, self.elevator, self.aileron, self.rudder])
                
                behavior = abs(self.vel_x) + abs(self.vel_z) + abs(self.pitch)*0.5 + \
                            abs(self.roll)*0.5 + abs(self.pitch_rate) + abs(self.roll_rate) + \
                                abs(self.ver_vel) + abs(self.dx) + abs(self.dz)
                    
                print(f'behavior: {behavior}\n')    
                if behavior <= 10 or self.has_crashed:
                    self.pitch_integral = 0
                    self.roll_integral = 0
                    self.yaw_integral = 0
                    target_altitude = 5
                    stage = 3
                elif behavior <= 12:
                    target_altitude = self.altitude + 10
            
            elif stage == 3:
                
                # accumulate error
                self.pitch_integral += self.pitch
                self.roll_integral += self.roll
                self.yaw_integral += (yaw_ref - self.yaw)

                # saturate integral
                self.pitch_integral = max(min(self.pitch_integral, integral_limit), -integral_limit)
                self.roll_integral = max(min(self.roll_integral, integral_limit), -integral_limit)
                self.yaw_integral = max(min(self.yaw_integral, integral_limit), -integral_limit)
                
                throttle_values = self.landing(target_altitude)
                
                self.starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                                "sim/flightmodel2/controls/pitch_ratio",
                                                "sim/flightmodel2/controls/roll_ratio",
                                                "sim/flightmodel2/controls/heading_ratio"],
                                               [throttle_values, self.elevator, self.aileron, self.rudder])
                # target_altitude -= 2
                # target_altitude = target_altitude if target_altitude > 5 else 5
                print(f'ver_vel: {self.ver_vel}\ntarget_altitude: {target_altitude}\n')
                
                safe_for_engine_turn_off = abs(self.altitude) + abs(self.vel_x) + abs(self.vel_z) + abs(self.pitch)*0.25 + \
                                            abs(self.roll)*0.25 + abs(self.pitch_rate) + abs(self.roll_rate) + abs(self.ver_vel)
                
                print(f'safe_for_engine_turn_off: {safe_for_engine_turn_off}')
                
                if (safe_for_engine_turn_off <=2.2) or self.has_crashed:
                    throttle_values = [0,0,0,0,0,0,0,0]
                    self.starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use",
                                                    "sim/flightmodel2/controls/pitch_ratio",
                                                    "sim/flightmodel2/controls/roll_ratio"],
                                                   [throttle_values, 0, 0])
                    
                    self.record_data = False

                    self.starship.sendDREF('sim/operation/override/override_throttles', 0)
                    self.starship.sendDREF('sim/operation/override/override_flightcontrol', 0)
                    break
        
        self.data_writer_event.set()
        save_data_to_csv(FLIGTH_DATA_DIR, FLIGTH_DATA_FILENAME, self.data)
        save_data_to_csv(FLIGTH_DATA_DIR, CONTROL_DATA_FILENAME, self.control_data)
        
        dfld = 'distance miss: ' + str(round(sqrt(((self.dx)**2)+((self.dz)**2)), 2)) + ' m'
        text = 'fuel cost: ' + str(round(self.starship.get_fuel_cost() * 100, 2)) + ' %'
        
        print(text)
        print(dfld)
        
        timestr = time.strftime("%Y-%m-%d-%H-%M-%S")
        file_name = 'FUEL_COST' + '_' + timestr + '.txt'
        exact_filename = os.path.join(FLIGTH_DATA_DIR, file_name)
        with open(exact_filename, 'w') as file:
            file.write(dfld)
            file.write('\n')
            file.write(text)
        
        print(f'\nrocket_enginge_off_alt: {self.rocket_enginge_off_alt}')
            
                
sim = XPlane()
sim.start_sim()
time.sleep(1)
sim.wait_threads()
