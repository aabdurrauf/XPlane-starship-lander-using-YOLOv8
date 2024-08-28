# -*- coding: utf-8 -*-
"""
Created on Fri Aug  2 16:50:36 2024

@author: Ammar Abdurrauf
"""

import cv2
import numpy as np
from time import time, sleep
from utils.utils import switch_tab, unpause_game, switch_to_fpv
from ultralytics.utils.plotting import Annotator
from wincam import DXCamera

from ultralytics import YOLO

path_to_model = "D:\\Projects\\xplane_yolo\\yolo\\m\\weights\\best.pt"
model = YOLO(path_to_model)


switch_tab()
sleep(0.5)

fps_list = []
loop_time = time()
with DXCamera(2, 103, 1278, 714, fps=100, capture_cursor=False) as camera:
    while True:
        
        frame, timestamp = camera.get_bgr_frame()
        frame = np.ascontiguousarray(frame)
            
        results = model(frame)
        
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
            max_conf *= 100
            annotator.box_label(b, "{:.2f}%".format(max_conf), color=(0, 0, 255))
            
        screenshot = annotator.result()
        
        display_screenshot = cv2.resize(screenshot, (639, 357))
        
        cv2.imshow('X-Plane 11 Screenshot', display_screenshot)
        
        fps = 1 / (time() - loop_time)
        fps_list.append(fps)
        print('FPS {}'.format(fps))
        loop_time = time()
        
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break


print('Done.')
print(f'avg: {sum(fps_list)/len(fps_list)}')
print(f'max: {max(fps_list)}')
        
# In[test]

boxes[0].xywh[0].tolist()[:2]
