# -*- coding: utf-8 -*-
"""
Created on Sat Aug 10 13:29:04 2024

@author: Ammar Abdurrauf
version: 1.0
"""

from ultralytics.utils.plotting import Annotator
from utils.utils import switch_tab, unpause_game
from src.starship import Starship
from ultralytics import YOLO
from wincam import DXCamera
import numpy as np
import threading
# import pyautogui
import time
import cv2

# constant
CENTER_X = 639
CENTER_Y = 357


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
        
    def set_env(self):
        switch_tab()
        time.sleep(0.5)
        
        # set the camera view
        # pyautogui.press('num9')
        self.starship.sendDREF('sim/graphics/view/view_type', 1025)
        
        # reposition the rocket
        self.starship.sendDREF('sim/flightmodel/position/local_x', -15531.20)
        self.starship.sendDREF('sim/flightmodel/position/local_z', -55350.81)
        
    def start_sim(self):
        self.set_env()
        time.sleep(0.5)
        self.object_detection_thread.start()
        time.sleep(0.5)        
        # throttle on
        self.starship.send_control(throttle=1.0)
        self.xplane_communication_thread.start()
        
    def wait_threads(self):
        self.object_detection_thread.join()
        self.xplane_communication_thread.join()
        
    def draw_crosshair(self, image):
        color = (100, 100, 100)  # Line color in BGR, e.g., blue
        thickness = 2  # Line thickness in pixels

        # draw the x-y coordinate
        y1_axis_start = (639, 0)  # Starting coordinate, e.g., (x1, y1)
        y1_axis_end = (639, 257)  # Ending coordinate, e.g., (x2, y2)
        
        y2_axis_start = (639, 457)  # Starting coordinate, e.g., (x1, y1)
        y2_axis_end = (639, 714)  # Ending coordinate, e.g., (x2, y2)
        
        x1_axis_start = (0, 357)  # Starting coordinate, e.g., (x1, y1)
        x1_axis_end = (539, 357)  # Ending coordinate, e.g., (x2, y2)
        
        x2_axis_start = (739, 357)  # Starting coordinate, e.g., (x1, y1)
        x2_axis_end = (1278, 357)  # Ending coordinate, e.g., (x2, y2)
        
        image = cv2.line(image, y1_axis_start, y1_axis_end, color, thickness)
        image = cv2.line(image, y2_axis_start, y2_axis_end, color, thickness)
        
        image = cv2.line(image, x1_axis_start, x1_axis_end, color, thickness)
        image = cv2.line(image, x2_axis_start, x2_axis_end, color, thickness)
        
        # draw the rectangle
        rect11_start = (539, 257)  # Starting coordinate, e.g., (x1, y1)
        rect11_end = (569, 257)  # Ending coordinate, e.g., (x2, y2)
        rect12_start = (539, 257)  # Starting coordinate, e.g., (x1, y1)
        rect12_end = (539, 287)  # Ending coordinate, e.g., (x2, y2)
        
        rect21_start = (709, 257)  # Starting coordinate, e.g., (x1, y1)
        rect21_end = (739, 257)  # Ending coordinate, e.g., (x2, y2)
        rect22_start = (739, 257)  # Starting coordinate, e.g., (x1, y1)
        rect22_end = (739, 287)  # Ending coordinate, e.g., (x2, y2)
        
        rect31_start = (539, 457)  # Starting coordinate, e.g., (x1, y1)
        rect31_end = (569, 457)  # Ending coordinate, e.g., (x2, y2)
        rect32_start = (539, 427)  # Starting coordinate, e.g., (x1, y1)
        rect32_end = (539, 457)  # Ending coordinate, e.g., (x2, y2)
        
        rect41_start = (709, 457)  # Starting coordinate, e.g., (x1, y1)
        rect41_end = (739, 457)  # Ending coordinate, e.g., (x2, y2)
        rect42_start = (739, 427)  # Starting coordinate, e.g., (x1, y1)
        rect42_end = (739, 457)  # Ending coordinate, e.g., (x2, y2)
        
        # cross on center
        cross1_start = (CENTER_X, 342)
        cross1_end = (CENTER_X, 372)
        
        cross2_start = (624, CENTER_Y)
        cross2_end = (654, CENTER_Y)
        
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
                target_center = None
                
                frame, timestamp = camera.get_bgr_frame()
                frame = np.ascontiguousarray(frame)
                print('frame:', frame.shape)
                
                results = self.yolo_model(frame)
                
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
                    
                    target_center = boxes[index].xywh[0].tolist()[:2]
                    
                    
                    
                screenshot = annotator.result()
                
                image = self.draw_crosshair(screenshot)
                image = self.draw_distance(image, target_center)
                
                display_screenshot = cv2.resize(image, (639, 357))
                
                cv2.imshow('Starship Landing', display_screenshot)
                
                print('FPS {}'.format(1 / (time.time() - loop_time)))
                loop_time = time.time()
                
                if cv2.waitKey(1) == ord('q'):
                    cv2.destroyAllWindows()
                    self.run = False
                    break
    
    def xplane_communication(self):
        landing = False
        landed = [False * 30]
        landed_index = 0
        self.starship.resume_sim()
        while self.run:
            values = self.starship.get_states()
            self.altitude = values[0]
            print('altitude:', self.altitude)
            if self.altitude > 200:
                self.starship.send_control(throttle=0)
                landing = True
            
            # pitch
            Pp = 0.04
            Dp = 0.004
            # roll
            Pr = 0.02
            Dr = 0.002
            # throttle
            Cv = 0.1
            Ca = 0.012
            
            pitch = values[4]
            pitch_rate = values[9]
            roll = values[5]
            roll_rate = values[10]
            ver_vel = values[3]
            
            elevator = -pitch * Pp + pitch_rate * Dp
            if elevator > 1:
                elevator = 1
            if elevator < -1:
                elevator = -1

            aileron = -roll * Pr + roll_rate * Dr
            if aileron > 1:
                aileron = 1
            if aileron < -1:
                aileron = -1
            
            if not landing:
                self.starship.send_control(elevator=elevator, aileron=aileron)
            else:
                throttle = min(max(-ver_vel * Cv - self.altitude * Ca, 0), 1)
                print('thorttle:', throttle)
                
                self.starship.send_control(throttle=throttle, elevator=elevator, aileron=aileron)
                
            if landing and self.starship.is_on_ground():
                landed[landed_index] = True
                landed_index += 1
                
            if False not in landed:
                self.starship.send_control(throttle=0.0)
                time.sleep(1)
                self.starship.pause_sim()
                pass
                
            

sim = XPlane()
sim.start_sim()
time.sleep(1)
sim.wait_threads()



