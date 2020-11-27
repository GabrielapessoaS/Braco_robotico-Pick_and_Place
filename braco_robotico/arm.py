#!/usr/bin/python3
import pigpio
import math

x = 17  # Pino do segmento AB
z = 4   # Pino do segmento BC
rb = 27 # Pino do servo da base

rpi = pigpio.pi()
rpi.set_mode(x, pigpio.OUTPUT)
rpi.set_mode(z, pigpio.OUTPUT)
rpi.set_mode(rb, pigpio.OUTPUT)

def pz(dz):
    p = 115*(180-dz)/9.0 + 200
    if p >= 2500:
        return 2490
    elif p <= 1000:
        return 1010
    else:
        return p

def px(dx):
    p = 22*dx/3.0 + 1280
    if p >= 1530:
        return 1520
    elif p <= 1000:
        return 1010
    else:
        return p


def getxy(a1, a2, d1, d2):
    x = d1*math.cos(a1*math.pi/180.0) + d2*math.cos(a2*math.pi/180.)

    y = d1*math.sin(a1*math.pi/180.0) + d2*math.sin(a2*math.pi/180.)
    print(x,y)

def setAngles(t1, t2):
    rpi.set_servo_pulsewidth(x, px(t2))
    rpi.set_servo_pulsewidth(z, pz(t1))

def setArm(x, y):
    d1 = 8.0
    d2 = 8.0
    q2 = math.acos((x*x + y*y - d1*d1 - d2*d2)/(2*d1*d2))
    q1 = math.atan(y/x)+math.atan(d2 * math.sin(q2)/(d1 + d2*math.cos(q2)))
    print("q1:", q1*180/math.pi, " q2:", q2*180/math.pi)
    theta1 = q1*180/math.pi
    theta2 = (q1-q2)*180/math.pi
    print("Theta 1:", theta1, " us:", pz(theta1))
    print("Theta 2:", theta2, " us", pz(theta2))
    print("ok")
    setAngles(theta1, theta2)


def reset():
    rpi.set_servo_pulsewidth(x, 0); 
    rpi.set_servo_pulsewidth(z, 0)
