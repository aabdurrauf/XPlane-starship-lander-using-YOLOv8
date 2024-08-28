# -*- coding: utf-8 -*-
"""
Created on Sat Jul 13 01:12:45 2024

@author: Ammar Abdurrauf
"""

import pandas as pd
import pyautogui
import time
import os

def switch_tab():
    pyautogui.hotkey('alt', 'tab')
        
def unpause_game():
    pyautogui.press('p')

def save_data_to_csv(file_dir, file_name, data):
    
    values = pd.DataFrame(list(zip(*data.values())),
                          columns=list(data.keys()))
    
    timestr = time.strftime("%Y-%m-%d-%H-%M-%S")
    file_name = file_name + '_' + timestr + '.xlsx'
    exact_filename = os.path.join(file_dir, file_name)

    with pd.ExcelWriter(exact_filename) as writer:
        values.to_excel(writer)
