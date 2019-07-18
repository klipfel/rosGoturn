#!/usr/bin/env python3

import board
import adafruit_pca9685
import busio
import time
import os

class Servomotor:
	
	def __init__(self):
		i2c = busio.I2C(board.SCL, board.SDA)
		self.pwm = adafruit_pca9685.PCA9685(i2c)
		self.pwm.frequency = 50
		# values
		self.leftMax = 100
		self.rightMax = 0
		self.straight = 50
		self.angle = None
		self.set_bearing(self.straight)
		print("Servomotor initialization SUCCESS")
	
	def test(self):
		accuracy = 10
		for angle in range(0,100 + accuracy,accuracy):
			self.set_bearing(angle)
			time.sleep(1)
		self.set_bearing(self.straight)
	
	def terminal_test(self):
		value = input("Angle [0-100]: ")
		while value != 'q':
			self.set_bearing(float(value))
			value = input("Angle [0-100]: ")
			print("value entered : " + value)
			
	def set_bearing(self,angle):
		self.pwm.channels[1].duty_cycle = int(3932+ angle*2620/100)
		self.angle = angle

class Motor:
	
	def __init__(self):
		i2c = busio.I2C(board.SCL, board.SDA)
		self.pwm = adafruit_pca9685.PCA9685(i2c)
		self.pwm.frequency = 50
		# values
		self.off = 50
		self.forwardMin = 68
		# motor setup
		self.setup()
		print("Motor initialization SUCCESS")

	def stop(self):
		self.set_speed(self.off)		

	def setup(self):
		self.set_speed(50)
		time.sleep(1)

	def test(self):
		for speed in range(0,100,10):
			self.set_speed(speed)
			time.sleep(1)
		self.set_speed(self.forwardMin)
		time.sleep(4)
		self.stop()
	
	def terminal_test(self):
		value = input("Speed [0-100]: ")
		while value != 'q':
			self.set_speed(float(value))
			value = input("Speed [0-100]: ")
			print("value entered : " + value)
	
	def set_speed(self,speed):
		self.pwm.channels[0].duty_cycle = int(3932+ speed*2620/100)

if __name__ == "__main__":
	print("test")
	b = Servomotor()
	m = Motor()
	b.test()
	m.terminal_test()
	
