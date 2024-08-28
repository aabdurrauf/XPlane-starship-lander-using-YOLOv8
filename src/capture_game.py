# -*- coding: utf-8 -*-
"""
Created on Fri Aug  2 17:13:11 2024

@author: Ammar Abdurrauf
"""

from utils.utils import switch_tab
import numpy as np
import cv2 as cv
import pyautogui
import time


# switch_tab()
# time.sleep(0.2)

screenshot = pyautogui.screenshot(region=(2, 66, 1280, 690))
screenshot = np.array(screenshot)
screenshot = cv.cvtColor(screenshot, cv.COLOR_RGB2BGR)


timestr = time.strftime("%Y%m%d-%H%M%S")
cv.imwrite(f'./landing_pad/landing_pad_{timestr}.jpg', screenshot)



# # In[2]

# switch_tab()
# time.sleep(0.2)
# screenshot = pyautogui.screenshot(region=(0,67, 1024, 768))

# timestr = time.strftime("%Y%m%d-%H%M%S")
# directory = r'D:\\Projects\\simplerockets2-test\\target_images'
# filename = f'target_{timestr}.jpg'

# cv.imwrite(filename, screenshot)    