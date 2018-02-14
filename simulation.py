#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 14 04:50:44 2018

@author: boatengyeboah
"""
import sys
sys.path.append("/Library/Frameworks/Python.framework/Versions/3.6/lib/python3.6/site-packages")

from vpython import *
from PID import PID

scene.title = "Fuzzy Logic Simulation"
scene.width = 1300
scene.height = 900

# Define simulation parameters
frame_rate = 20
delta_t = 1/frame_rate 
t = 0


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
cart = box(pos=vector(0,platform_width,0), size=vector(cart_length, cart_width, cart_height))

# Define physics parameters
g = 9.81
cart.mass = 5 # 5kg
cart.v = vector(0,0,0) # Initial cart velocity = 0 m/s
accel = vector(0,0,0) # Acceleration vector of cart
cart.dist = 0

# cart physics properties
cart_accel_vector = arrow(pos=vector(0,0,0),axis=platform.axis.norm(), shaftwidth=0.01)
# Time label
time_label = label( pos=vec(0, 0.5, 0), text='Time = 0 sec' )
platform_angle_label = label( pos=vec(0, 0.45, 0), text='Theta = 0 deg' )
sensor_reading_label = label( pos=vec(0, 0.4, 0), text='Sensor = +0 m' )

simulation_running = True

# PID controller test
pid = PID(0.07, 0.01, 0.0)
pid.setSampleTime(1/frame_rate)
pid.SetPoint = 0.3 # Set setpoint to 0.5m
max_pid_output = radians(30)
min_pid_output = -radians(30)


prev_theta = 0

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
    global theta
    global d_theta, hitLeft, hitRight
    pid.update(cart.dist)
   
    prev_theta = theta
    theta = capPidOutput(-pid.output)
    d_theta = theta - prev_theta
    #print("Current = {:.2f}, Error = {:.2f},Theta={}, Prev={}, dTheta={}".format(cart.dist, pid.SetPoint-cart.dist, degrees(theta),degrees(prev_theta), degrees(d_theta)))
    #theta = -testTheta
    #theta += d_theta
    accel  = norm(platform.axis) * -1
    accel.mag = g * sin(theta) 
    print("Cart dist= {}, Accel={}".format(cart.dist, accel))
    cart.v += accel * delta_t
    if cart.pos.x <= (-platform_length/2)*cos(theta) and not hitLeft:
        hitLeft = True
        cart.v *= -1
    elif cart.pos.x > (-platform_length/2)*cos(theta):
        hitLeft = False
            
            
    if cart.pos.x >= (platform_length/2)*cos(theta)-0.05 and not hitRight:
        hitRight = True
        print("Inside positive")
        cart.v *= -1
    elif cart.pos.x < (platform_length/2)*cos(theta)-0.05:
        hitRight = False
            #cart.pos.y = (-cart.pos.x)*sin(theta) + cart_width
            
    cart.pos += cart.v * delta_t
    cart.pos.y = cart.pos.x * sin(theta)# + cart_width
    cart.dist = cart.pos.mag
    
    if cart.pos.x < 0:
        cart.dist *= -1
    #print("Cart x = {:.2f}, Accel ={}".format(cart.pos.x, accel))
     #d_theta = testTheta
    
    
    
     
theta = 0
with open('plot.csv', 'w') as f:
    while simulation_running:
        rate(frame_rate)
        #theta += d_theta
        computeForces()
        rotatePlane()
        showLabels()
        #f.write("{},{},{}\n".format(cart.dist, pid.SetPoint-cart.dist, t))       
        if theta < -radians(max_platform_angle):
            d_theta *= -1
    
        if theta > radians(max_platform_angle):
            d_theta *= -1
        t += delta_t
    
    
    
    
    
    
    
    
    
    
    
    
