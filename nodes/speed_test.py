#!/usr/bin/env python3
''' simulation of the ROS node of the motor.'''
from speed_controller import *

if __name__ == "__main__":
	print("Motor Node")
	motor = Motor()
	motor.terminal_test()
