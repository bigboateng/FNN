from __future__ import division
from visual import *
scene.title = "Fuzzy Neuro Network"

scene.background = color.black
scene.center = (0, 1, 0) # location at which the camera looks
scene.width = 1000
scene.height = 1000
maxAngle = 10
maxAngleTime = 3
frameRate = 30
dTheta = ((maxAngle/maxAngleTime)/frameRate) * (pi / 180.0)

# Define scene objects (units are in meters)

# 1.4-m long inclined plane whose center is at 0.7 m
planeLength = 2 # 2m
inclinedPlane = box(pos = vector(0, 0, 0), size = (planeLength, 0.02, 0.2),
color = color.green, opacity = 0.3)
# 20-cm long cart on the inclined plane
cartLength = 0.2
cart = box(size = (cartLength, 0.06, 0.06), color = color.blue)
# Define parameters
cart.m = 5 # mass of cart in kg

# initial position of the cart in(x, y, z) form, units are in meters
#   cart is positioned on the inclined plane at the far left end
cart.pos = vector(0, 0.04, 0.08)

cart.v = vector(0, 0, 0) # initial velocity of car in (vx, vy, vz) form, units are m/s

# angle of inclined plane relative to the horizontal
theta = 0

# rotate the cart and the inclined plane based on the specified angle (counterclockwise)
inclinedPlane.rotate(angle = 0, origin = inclinedPlane.pos, axis = (0,0,1))
cart.rotate(angle = theta, origin = (0, 0, 0), axis = (0,0,1))

# set the initial velocity up the ramp; units are m/s
cart.v = norm(inclinedPlane.axis)
cart.v.mag = 0

g = 9.8 # acceleration due to gravity; units are m/s/s

mu = 2e-4# coefficient of friction between cart and plane

# Define time parameters
t = 0 # starting time
deltat = 1/frameRate  # time step units are s


### CALCULATION LOOP; perform physics updates and drawing
# ------------------------------------------------------------------------------------

#pointer = arrow(pos=(0,0,0),axis=inclinedPlane.axis.norm(), shaftwidth=0.1)
angle = 0

fNetDir= arrow(pos=(0,0,0),axis=(0,0,1), shaftwidth=0.01)

f = open("points.csv", 'w')

accel = vector(0,0,0)
flag = 0
with open("points.csv", 'w') as f:
    while True:  # while the cart's y-position is greater than 0 (above the ground)
        rate(frameRate)    
    
        inclinedPlane.rotate(angle = dTheta, origin = (0,0,0), axis = (0,0,1))

        # Compute Net Force 

        Fnet = norm(inclinedPlane.axis)
        # set the magnitude to the component of the gravitational force parallel to the inclined plane
        Fnet.mag = -(cart.m * g * sin(theta)) #- (mu * cart.m * g * cos(theta))

        
        accel  = norm(inclinedPlane.axis) * -1
        accel.mag = g * sin(theta) 

        if theta > 0:
            accel.mag += mu * g * cos(theta)
        else:
            accel.mag -= mu * g * cos(theta)
        
        fNetDir.axis = Fnet.norm()
        cart.v = cart.v + (accel* deltat) 
    
        print("Accel = {}, Pos = {}, planeX={}".format(accel, cart.pos, (planeLength/2)*cos(theta)))
        
        if accel.x < 0:
            if cart.pos.x <= (-planeLength/2)*cos(theta):
                cart.v = vector(0,0,0)
                #cart.rotate(angle = dTheta, origin = (0,0,0), axis = (0,0,1))
            

        if accel.x > 0:
            if cart.pos.x >= (planeLength/2)*cos(theta)-0.05:
                cart.v = vector(0,0,0)
                #cart.rotate(angle = dTheta, origin = (0,0,0), axis = (0,0,1))
                
        if (theta < -maxAngle* (pi / 180.0)):
            dTheta *= -1

        if theta > maxAngle* (pi / 180.0):
            print("Inside theta")
            dTheta *= -1

        cart.pos = cart.pos + cart.v * deltat
        theta += dTheta
        t = t + deltat     
