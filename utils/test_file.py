# -*- coding: utf-8 -*-
"""
Created on Sun Aug 18 11:51:29 2024

@author: ammar
"""
from src.starship import Starship
from utils.utils import switch_tab
from math import pi, cos, sin
import time
import pyautogui

starship = Starship()

starship.client.getDREF('sim/flightmodel/weight/m_fuel1')[0]

switch_tab()
time.sleep(0.2)
pyautogui.keyDown('shift')
pyautogui.press('4')
pyautogui.keyUp('shift')

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

# states = starship.get_states()

starship.sendDREF('sim/graphics/view/view_type', 1025)
# time.sleep(0.2)
# starship.sendDREF('sim/view/free_camera', True)

starship.set_position(true_head=90)
# set the starship heading
starship.sendDREF('sim/flightmodel/position/q', [0.8, 0, 0, 0.82])

starship.sendDREF('sim/flightmodel/position/local_z', 
                  starship.client.getDREF('sim/flightmodel/position/local_z')[0]+100)

starship.sendDREF('sim/flightmodel/position/local_x', 
                  starship.client.getDREF('sim/flightmodel/position/local_x')[0]+10)

starship.sendDREF('sim/flightmodel/position/local_x', -15531.15)
starship.sendDREF('sim/flightmodel/position/local_z', -55350.81)

starship.client.getDREF('sim/flightmodel/position/local_x')
starship.client.getDREF('sim/flightmodel/position/local_y')
starship.client.getDREF('sim/flightmodel/position/local_z')

starship.client.getPOSI()

starship.sendDREF('sim/graphics/view/view_type', 1025)
starship.sendDREF('sim/flightmodel/position/local_y', 
                  starship.client.getDREF('sim/flightmodel/position/local_y')[0]+200)
x = -3
for i in range(60):
    x += 0.1
    print(f'x: {x}')
    starship.sendDREF('sim/flightmodel/position/q', [1, 0, 0, x])
    time.sleep(0.2)

# In[]
starship.get_states()
starship.set_position(true_head=90)
starship.set_position(latitude=26.00926, longitude=-98.22555, altitude=64.1)
# In[]
from src.starship import Starship
from utils.utils import switch_tab
from math import pi, cos, sin
import time

starship = Starship()

q = [0]*4

desired_heading = 90

starship.client.clearBuffer()
psi_ = starship.client.getDREF('sim/flightmodel/position/psi')[0]
theta_ = starship.client.getDREF('sim/flightmodel/position/theta')[0]
phi_ = starship.client.getDREF('sim/flightmodel/position/phi')[0]
print(f'psi: {psi_}\ntheta; {theta_}\nphi: {phi_}\n')
starship.client.clearBuffer()
    
psi = pi / 360 * (desired_heading)
theta = pi / 360 * theta_
phi = pi / 360 * phi_
q[0] =  cos(psi) * cos(theta) * cos(phi) + sin(psi) * sin(theta) * sin(phi)
q[1] =  cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta) * cos(phi)
q[2] =  cos(psi) * sin(theta) * cos(phi) + sin(psi) * cos(theta) * sin(phi)
q[3] = -cos(psi) * sin(theta) * sin(phi) + sin(psi) * cos(theta) * cos(phi)

starship.sendDREF('sim/flightmodel/position/q', q)
print(starship.client.getDREF('sim/flightmodel/position/psi')[0])

# In[]

# starship.sendDREF('sim/flightmodel/position/local_x', -15531.20)
# starship.sendDREF('sim/flightmodel/position/local_z', -55350.81)

# starship.sendDREF('sim/flightmodel/position/local_x', 
#                   starship.client.getDREF('sim/flightmodel/position/local_x')[0]-10)
# starship.sendDREF('sim/flightmodel/position/local_z', 
#                   starship.client.getDREF('sim/flightmodel/position/local_z')[0]+10)
starship.sendDREF('sim/flightmodel/position/local_y', 
                  starship.client.getDREF('sim/flightmodel/position/local_y')[0]+1000)

starship.sendDREF('sim/operation/override/override_flightcontrol', 1)
starship.sendDREF('sim/operation/override/override_throttles', 1)


rudder = 0.1
symb = 1
prev_heading = 0
while not starship.has_crashed():
    
    heading = starship.client.getDREF('sim/flightmodel/position/q')
    while prev_heading == sum(heading):
        heading = starship.client.getDREF('sim/flightmodel/position/q')
    
    prev_heading = sum(heading)
    
    starship.client.sendDREF("sim/flightmodel2/controls/heading_ratio", rudder)
    rudder += (symb * 0.1)
    if abs(rudder) >= 1.28:
        symb = (-1) * symb
        
    t3 = 0
    t4 = 0
    t5 = 0
    t6 = 0
    
    th = [0.6,0.6,0.6,t3,t4,t5,t6,0]
    starship.client.sendDREF("sim/flightmodel/engine/ENGN_thro_use", th)
    print(f'rudder: {rudder}\nheading: {heading}\n')
    time.sleep(0.1)
    
starship.sendDREF('sim/operation/override/override_throttles', 0)
starship.sendDREF('sim/operation/override/override_flightcontrol', 0)

# starship.send_control(elevator=0, aileron=0)
# starship.sendDREF('sim/graphics/view/view_type', 1015)

# starship.sendDREF('sim/flightmodel/position/local_vz', 15)

# starship.sendDREF('sim/flightmodel/weight/m_fuel1', 293035)

import time
t3 = 0
t4 = 0
t5 = 0
t6 = 0

th = [0,0,0,t3,t4,t5,t6,0]
starship.client.sendDREF("sim/flightmodel/engine/ENGN_thro_use", th)
starship.sendDREF('sim/operation/override/override_throttles', 0)

time.sleep(1)
th = [0,0,0,0,0,0,0]
starship.client.sendDREF("sim/flightmodel/engine/ENGN_thro_use", th)

# starship.client.sendDREFs(["sim/flightmodel/engine/ENGN_thro_use[0]", "sim/flightmodel/engine/ENGN_thro_use[1]"], [0, 1])

# starship.client.sendDREF("sim/flightmodel/engine/ENGN_thro_use[]", [0, 0, 0, 0, 0, 0, 0, 0])
data = starship.client.getDREF("sim/flightmodel/engine/ENGN_thro_use")


starship.resume_sim()


# In[]

from src.starship import Starship
import pyautogui
from utils.utils import switch_tab
import time

starship = Starship()
switch_tab()
time.sleep(0.2)
pyautogui.keyDown('shift')
pyautogui.press('4')
pyautogui.keyUp('shift')

i = 0
time.sleep(0.2)
# with pyautogui.hold('shift'):
while i < 30:
    i += 1
    pyautogui.keyDown(',')
pyautogui.keyUp(',')

i = 0
while i < 5:
    i += 1
    pyautogui.keyDown('up')
pyautogui.keyUp('up')

i = 0
while i < 4:
    i += 1
    pyautogui.keyDown('left')
pyautogui.keyUp('left')


starship.sendDREF('sim/flightmodel/position/local_y', 
                  starship.client.getDREF('sim/flightmodel/position/local_vz')[0] + 2000)
i = 0
while True:
    starship.sendDREF('sim/operation/override/override_throttles', 1)
    starship.sendDREF('sim/operation/override/override_flightcontrol', 1)
    th = [0.4,0.4,0.4,0,0,0,0,0]
    starship.client.sendDREF("sim/flightmodel/engine/ENGN_thro_use", th)
    
    if i % 2 == 0:
        v = [70, 60, 90, 0, 180, 0, 0, 0]
        # starship.sendDREF('sim/operation/override/override_throttles', 0)
        starship.client.sendDREF("sim/flightmodel2/controls/roll_ratio", 0.5)
        time.sleep(1)
        # sim/flightmodel2/engines/rotor_vertical_vector_deg
    else:
        v = [90, 80, 70, 0, 180, 0, 0, 0]
        # starship.sendDREF('sim/operation/override/override_throttles', 0)
        # starship.client.sendDREF("sim/flightmodel2/controls/pitch_ratio", -0.8)
    
    i += 1
    crash = starship.has_crashed()
    if crash:
        starship.sendDREF('sim/operation/override/override_throttles', 0)
        starship.sendDREF('sim/operation/override/override_flightcontrol', 0)
        break



# In[]



from utils.utils import save_data_to_csv
from src.constants10 import FLIGTH_DATA_FILENAME, FLIGTH_DATA_DIR
import pandas as pd

data = {'altitude': [], 'ver vel': [], 'pitch': []}

data['altitude'].append(1)
data['altitude'].append(3)
data['altitude'].append('sls')
data['altitude'].append(5)
data['altitude'].append(1)
data['altitude'].append(22)
data['altitude'].append(12)
data['altitude'].append(76)
data['altitude'].append('msms')
data['altitude'].append(5)
data['altitude'].append(1)
data['altitude'].append(3)
data['altitude'].append('sls')
data['altitude'].append(5)
data['altitude'].append(1)
data['altitude'].append(3)
data['altitude'].append('sls')
data['altitude'].append(5)

data['ver vel'].append(4)
data['ver vel'].append(-3)
data['ver vel'].append(-5)
data['ver vel'].append(-5)
data['ver vel'].append(None)
data['ver vel'].append(-5)
data['ver vel'].append(4)
data['ver vel'].append(-3)
data['ver vel'].append(-5)
data['ver vel'].append(-5)
data['ver vel'].append(4)
data['ver vel'].append(-3)
data['ver vel'].append(-5)
data['ver vel'].append(-5)
data['ver vel'].append(None)
data['ver vel'].append(-5)
data['ver vel'].append(4)
data['ver vel'].append(-3)
data['ver vel'].append(-5)
data['ver vel'].append(-5)
data['ver vel'].append(None)
data['ver vel'].append(-5)

data['pitch'].append(10.2)
data['pitch'].append(-0.2)
data['pitch'].append(-2.1)
data['pitch'].append(-2.1)
data['pitch'].append(-2.1)
data['pitch'].append(-2.1)
data['pitch'].append(-0.2)
data['pitch'].append(-2.1)
data['pitch'].append(10.2)
data['pitch'].append(-0.2)
data['pitch'].append(-2.1)
data['pitch'].append(-2.1)
data['pitch'].append(10.2)
data['pitch'].append(-0.2)
data['pitch'].append(-2.1)
data['pitch'].append(-2.1)
data['pitch'].append(10.2)
data['pitch'].append(-0.2)
data['pitch'].append(-2.1)
data['pitch'].append(-2.1)

print(f"\n--------\ndata info:\ndata len: {len(data['altitude'])}\ndata: {data['altitude'][:10]}\n-----\n")

save_data_to_csv(FLIGTH_DATA_DIR, FLIGTH_DATA_FILENAME, data)



# values = pd.DataFrame(list(zip(*data.values())), columns=list(data.keys()))
# values['altitude']

# In[]
from src.constants10 import CENTER_X, CENTER_Y, LANDING_CENTER_X, LANDING_CENTER_Z
from src.constants10 import INITIAL_POS_X, INITIAL_POS_Z, MAX_ALTITUDE, DIST_X, DIST_Z
import cv2
from ultralytics.utils.plotting import Annotator
from ultralytics import YOLO
from wincam import DXCamera
import numpy as np
from src.starship import Starship
from utils.utils import switch_tab
from math import pi, cos, sin
import time
import pyautogui

starship = Starship()

def draw_crosshair(image):
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

def draw_distance(image, target_pos=None):
    
    if target_pos is not None:
        color = (255, 0, 0)  # Line color in BGR, e.g., blue
        thickness = 2  # Line thickness in pixels
        
        line_start = (CENTER_X, CENTER_Y)
        line_end = (int(target_pos[0]), int(target_pos[1]))
        
        image = cv2.line(image, line_start, line_end, color, thickness)
        
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
        print(f'x: {x}\ny: {y}\n')
    return image

yolo_model_path="D:\\Projects\\xplane_yolo\\yolo\\n\\weights\\best.pt"
yolo_model = YOLO(yolo_model_path)

starship.set_position(true_head=90)

dx = 0
dz = 0
dy = 100 # 200, 500, 1000
starship.sendDREF('sim/flightmodel/position/local_x', -15531.15 + dx)
starship.sendDREF('sim/flightmodel/position/local_z', -55350.81 + dz)
starship.sendDREF('sim/flightmodel/position/local_y', -237.4 + dy)

starship.sendDREF('sim/graphics/view/view_type', 1025)

radius = 8.2
# starship.client.getDREF('sim/flightmodel2/position/y_agl')[0] * 0.3048


starship.client.getDREF('sim/flightmodel/position/local_y')[0]

xplane_screen = [2, 103, 1278, 714]
with DXCamera(xplane_screen[0],
              xplane_screen[1],
              xplane_screen[2],
              xplane_screen[3], 
              capture_cursor=False,
              fps=30) as camera:
    
    while True:
        
        frame, timestamp = camera.get_bgr_frame()
        frame = np.ascontiguousarray(frame)
        
        results = yolo_model(frame, verbose=False)
        
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
            try:
                annotator.box_label(b, "dz: {:.2f} m".format(0), color=(0, 0, 255))
            except TypeError:
                annotator.box_label(b, "dz: 0.0 m", color=(0, 0, 255))
            
            target_center = boxes[index].xywh[0].tolist()[:2]
            
            print('height:', boxes[index].xywh[0].tolist()[-1])
            
            # keep the previous target center coordinate
        
        else:
            target_center = None
            
            
        screenshot = annotator.result()
        
        image = draw_crosshair(screenshot)
        image = draw_distance(image, target_center)
        
        display_screenshot = cv2.resize(image, (639, 357))
        
        cv2.imshow('Starship Landing', display_screenshot)
        
        # print('FPS {}'.format(1 / (time.time() - loop_time)))
        loop_time = time.time()
        
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break








