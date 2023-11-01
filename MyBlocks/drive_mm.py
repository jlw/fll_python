#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
leftMotor = Motor(B, positive_direction=Direction.CLOCKWISE, gears=None)
rightMotor = Motor(C, positive_direction=Direction.CLOCKWISE, gears=None)
driveBase = DriveBase(leftMotor, rightMotor, 87, 119)

# Write your program here.
def drive_mm(angle, speed, mm, brake=True):
  driveBase.reset()
  while driveBase.distance() < mm:
    driveBase.drive(speed, angle)
  driveBase.stop()
  if brake == True:
    leftMotor.hold()
    rightMotor.hold()
  elif brake == False:
    leftMotor.brake()
    rightMotor.brake()