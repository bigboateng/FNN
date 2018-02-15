#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 14 04:50:44 2018

@author: boatengyeboah
"""
import sys
sys.path.append("/Library/Frameworks/Python.framework/Versions/3.6/lib/python3.6/site-packages")

from vpython import *
from random import randint
from PID import PID
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
from Fuzzy import Fuzzy
from anfis import *
from mfDerivs import *
from membershipfunction import *

# End of widgets
# Define simulation parameters
frame_rate = 20
delta_t = 1/frame_rate 
t = 0

# ANFIS setupg
ts = np.loadtxt("trainingdata1.text")
mf = mf = [[['gaussmf',{'mean':-11.,'sigma':5.}],['gaussmf',{'mean':-8.,'sigma':5.}],['gaussmf',{'mean':-14.,'sigma':20.}],['gaussmf',{'mean':-7.,'sigma':7.}]]]
X = ts[:,0]
Y = ts[:,1]

mfc = MemFuncs(mf)
#anf = ANFIS(X, Y, mfc)



# Array to keep track of setpoints
error_counter = np.zeros(frame_rate*10)

def push(a, n):
     a = np.roll(a, 1)
     a[0] = n
     return a
 
error_counter = push(error_counter, 100)
# Possibkle setpoints array
setpoints_list = np.linspace(-0.8, 0.8, 5)
setpoints_list_length = len(setpoints_list)
print("Number of setpoints = " + str(setpoints_list_length))


#Plotting
oscillation = graph(title="PID Response", xtitle='time', ytitle='value', fast=False, width=800)
funct1 = gcurve(color=color.blue, width=4, markers=True, marker_color=color.orange, label='curve')



scene.title = "Fuzzy Logic Simulation"
scene.width = 700
scene.height = 700



# PID controller test
#pid = PID(0.07, 0.001, 0.01)
#pid = PID(0.07, 0.0009, 0.04)
pid = PID(0.07, 0.0009, 0.07)
pid.setSampleTime(1/frame_rate)
pid.SetPoint = 0.45 # Set setpoint to 0.5m
max_pid_output = radians(30)
min_pid_output = -radians(30)


# Fuzzy logic controller
fuzzy_controller = Fuzzy()

# Setpoint text
setPText = wtext(text="HELLO") 
# Widgets
def S(s): 
    global pid
    global setPText
    pid.SetPoint = s.value
    setPText.text = str(s.value)+ "m"
    #print(s.value)
slider(bind=S, min=-0.5, max=0.5)
scene.append_to_caption('\n\n')


# Testing platform rotation
max_platform_angle = 45
max_platform_angle_time = 4
d_theta = (max_platform_angle/max_platform_angle_time)/frame_rate
d_theta = radians(d_theta)
theta = 0

# Define Scene Objects
platform_length = 2 # 2m
platform_width = 0.02
platform_height = 0.2
platform = box(pos=vector(0,0,0), size=vector(platform_length, platform_width, platform_height), color=vector(255, 0, 0))
# plaform base
plaform_base = box(pos=vector(0,-0.25, 0), size=vector(0.02, 0.5, 0.02), color=vector(139, 0, 0))

# Stop block right and left
stop_block_width = 0.1
stop_block_length = 0.1
stop_block_right = box(pos=vector((platform_length/2)+(stop_block_width),0,0), size=vector(stop_block_length,stop_block_width, stop_block_width))
stop_block_left = box(pos=vector((-platform_length/2)-(stop_block_width),0,0), size=vector(stop_block_length,stop_block_width, stop_block_width))
# Cart 
cart_length = 0.2
cart_width = 0.02
cart_height = 0.2
cart_start_x = -0.4
cart = box(pos=vector(cart_start_x,platform_width,0), size=vector(cart_length, cart_width, cart_height))

# Define physics parameters
g = 9.81
cart.mass = 5 # 5kg
cart.v = vector(0,0,0) # Initial cart velocity = 0 m/s
accel = vector(0,0,0) # Acceleration vector of cart
cart.dist = 0

# cart physics properties
cart_accel_vector = arrow(pos=vector(0,0,0),axis=platform.axis.norm(), shaftwidth=0.01)
# Time label
time_label = label( pos=vec(0, 0.7, 0), text='Time = 0 sec' )
platform_angle_label = label( pos=vec(0, 0.5, 0), text='Theta = 0 deg' )
sensor_reading_label = label( pos=vec(0, 0.35, 0), text='Sensor = +0 m' )

simulation_running = True




prev_theta = 0



# Flags to detect whether cart got to ends 

hitLeft = False
hitRight = False
def capPidOutput(current_output):
    if current_output >= max_pid_output:
        return max_pid_output
    
    if current_output <= min_pid_output:
        return min_pid_output
    
    return current_output



def showLabels():
    time_label.text = "Time elapsed: {:.2f} sec".format(t)
    platform_angle_label.text = "Theta = {:.2f} deg".format(degrees(theta))
    sensor_reading_label.text = "Sensor = {:.2f} m".format(cart.dist)

def rotatePlane():
     platform.rotate(angle = d_theta, origin = vector(0,0,0), axis = vector(0,0,1))
     stop_block_right.rotate(angle = d_theta, origin = vector(0,0,0), axis = vector(0,0,1))
     stop_block_left.rotate(angle = d_theta, origin = vector(0,0,0), axis = vector(0,0,1))
     cart.rotate(angle = d_theta, origin = vector(0,0,0), axis = vector(0,0,1))
     
     
def computeForces():
    global theta, error_counter, setpoints_list_length
    global d_theta, hitLeft, hitRight, fuzzy_controller, cart
    pid.update(cart.dist)
   
    prev_theta = theta
    theta = capPidOutput(-pid.output)
    #theta = -fuzzy_controller.compute(pid.SetPoint - cart.dist)
    #print(theta)
    d_theta = theta - prev_theta
    #print("Current = {:.2f}, Error = {:.2f},Theta={}, Prev={}, dTheta={}".format(cart.dist, pid.SetPoint-cart.dist, degrees(theta),degrees(prev_theta), degrees(d_theta)))
    #theta = -testTheta
    #theta += d_theta
    accel  = norm(platform.axis) * -1
    accel.mag = g * sin(theta) 
    #print("Cart dist= {}, Accel={}".format(cart.dist, accel))
    cart.v += accel * delta_t
    if cart.pos.x <= (-platform_length/2)*cos(theta) and not hitLeft:
        hitLeft = True
        cart.v *= -1
    elif cart.pos.x > (-platform_length/2)*cos(theta):
        hitLeft = False
            
            
    if cart.pos.x >= (platform_length/2)*cos(theta)-0.05 and not hitRight:
        hitRight = True
        #print("Inside positive")
        cart.v *= -1
    elif cart.pos.x < (platform_length/2)*cos(theta)-0.05:
        hitRight = False
            #cart.pos.y = (-cart.pos.x)*sin(theta) + cart_width
            
    cart.pos += cart.v * delta_t
    cart.pos.y = cart.pos.x * sin(theta)# + cart_width
    cart.dist = cart.pos.mag
    
    if cart.pos.x < 0:
        cart.dist *= -1
            
    error_counter = push(error_counter, abs(pid.SetPoint - cart.dist))
    if round(abs(np.average(error_counter)), 2) == 0:
        next_setpoints_position = randint(1,setpoints_list_length)
        pid.SetPoint = setpoints_list[next_setpoints_position]
        print("Setpoint = " + str(pid.SetPoint))
        
    if round(abs(pid.SetPoint - cart.dist), 2) == 0:
        cart.color = color.blue
    else:
        cart.color = color.white
    #print("Cart x = {:.2f}, Accel ={}".format(cart.pos.x, accel))
     #d_theta = testTheta
    
    
    
     
theta = 0
import os
#os.remove("plot.csv")

import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP





with open('trainingdata.text', 'w') as f:
    while simulation_running:
        rate(frame_rate)
        computeForces()
        rotatePlane()
        showLabels()
        #f.write("{} \t {}\n".format(pid.SetPoint - cart.dist, theta))
        funct1.plot(t, cart.dist)
        if theta < -radians(max_platform_angle):
            d_theta *= -1
    
        if theta > radians(max_platform_angle):
            d_theta *= -1
        t += delta_t
    
    
    
    
    
    
    
    
    
    
    
    
