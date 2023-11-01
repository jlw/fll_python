#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import LightSensor

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

from generic_robot import GenericRobot

# Create your objects here.
ev3 = EV3Brick()
leftMotor = Motor(Port.B, Direction.CLOCKWISE)
rightMotor = Motor(Port.C, Direction.CLOCKWISE)
driver = DriveBase(leftMotor, rightMotor, 87, 119)
inf = InfraredSensor(Port.S2)

leftColor = LightSensor(Port.S1)
rightColor = LightSensor(Port.S3)

myblocks = GenericRobot(ev3, driver, leftMotor, rightMotor, leftColor, rightColor, inf)
myblocks.drive_mm(0, 1000, 100)
