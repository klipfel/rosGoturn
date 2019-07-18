#!/usr/bin/env python3
''' simulation of the ROS node of the servomotor.'''
from speed_controller import *

if __name__ == "__main__":
	print("Servomotor Node")
	servo = Servomotor()
	servo.terminal_test()
