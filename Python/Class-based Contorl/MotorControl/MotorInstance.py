# -*- coding: utf-8 -*-
"""
Created on Fri Jul 11 13:48:01 2025

@author: USER
"""

#%% Import Libraries
import sys
from LogControlManager import LogControlManager

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python motor_instance.py <motor_id> <port> <baudrate>")
        sys.exit(1)

    motor_id = int(sys.argv[1])
    port     = sys.argv[2]
    baudrate = int(sys.argv[3])
    
    manager = LogControlManager(port=port, baudrate=baudrate, motor_id=motor_id)
    manager.send_id()
    manager.command_loop()