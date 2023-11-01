#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import LightSensor

#from main import Generic_Myblocks

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
leftMotor = Motor(Port.B, Direction.CLOCKWISE, gears=None)
rightMotor = Motor(Port.C, Direction.CLOCKWISE, gears=None)
driver = DriveBase(leftMotor, rightMotor, 87, 119)
inf = InfraredSensor(Port.S2)

# Write your program here.
class PIDController:
  def __init__(self):
    self.integral = 0
    self.last_error = 0

  def adjust(error, gainP, gainI, gainD):
    self.integral = error + self.integral
    derivative = error - self.last_error
    return (error * gainP) + (self.integral * gainI) + (derivative * gainD)

class Generic_Myblocks:
  def __init__(self, EV3Brick, DriveBase, LeftMotor, RightMotor, LightSensorLeft, LightSensorRight, GyroSensor):
    self.ev3 = EV3Brick
    self.robot = DriveBase
    self.lm = LeftMotor
    self.rm = RightMotor
    self.sen1 = LightSensorLeft
    self.sen2 = LightSensorRight
    self.gyro = GyroSensor
    self.stopWatch = StopWatch()


  def drive_mm(self, angle, speed, mm, rate=500, brake=True):
    self.robot.reset()
    if angle != 0:
      self.robot.settings(speed, rate)
      self.robot.straight(mm)
    else:
      while self.robot.distance() < mm:
        self.robot.drive(speed, angle)
    self.robot.stop()
    self.lm.hold()
    self.rm.hold()
  
  def pivot(self, angle, speed):
    self.robot.settings(2000, 5000, speed, 3000)
    self.robot.turn(angle)
    self.robot.stop()
    self.lm.hold()
    self.rm.hold()

  def black_line_square(self, target, targetBlack, targetWhite, approachSpeed, finetuneSpeed):
    def __waitUntil(sensor, input1):
      while sensor.reflection() <= input1:
        wait(0)
    def __colorIsInRange(value):
      if targetBlack <= value and targetWhite >= value:
        return True
      else:
        return False
    # Initiate Variables
    ev3.light.on(Color.RED)
    stopWatch.pause()
    stopWatch.reset()
    stopWatch.resume()
    reverseFinetuneSpeed = (0 - finetuneSpeed)
    ref1 = sen1.reflection()
    ref2 = sen2.reflection()
    # Roughly lines up with the black line
    robot.drive(approachSpeed)
    while True:
      if sen1.reflection() <= target:
        robot.stop()
        lm.hold()
        rm.run(approachSpeed)
        waitUnitl(sen2, target)
        rm.hold()
        break
      elif sen2.reflection() <= target:
        robot.stop()
        rm.hold()
        lm.run(approachSpeed)
        waitUntil(sen1, target)
        lm.hold()
        break
    # Sets status light to Yellow for fine tuning
    ev3.light.on(Color.YELLOW)
    while (colorIsInRange(ref1) and colorIsInRange(ref2)) or stopWatch.time() == returnTime:
      # Fine tunes the left motor
      if sen1.reflection() < targetBlack:
        lm.run(reverseFinetuneSpeed)
      elif sen1.reflection() > targetWhite:
        lm.run(finetuneSpeed)
      else:
        lm.hold()
      # Fine tunes the right motor
      if sen2.reflection() < targetBlack:
        rm.run(reverseFinetuneSpeed)
      elif sen2.reflection() > targetWhite:
        rm.run(finetuneSpeed)
      else:
        rm.hold()
      ref1 = sen1.reflection()
      ref2 = sen2.reflection()
    ev3.light.on(Color.GREEN)

  def gyro_drive(self, angle, speed, distance_mm, gainP, gainI, gainD, reset_sensor):
    if reset_sensor == False:
      gyro.reset_angle(0)
    pid_controller = PIDController()
    while robot.distance() < distance_mm:
      robot.drive(speed, pid_controller.adjust(angle - gyro.angle(), gainP, gainI, gainD))
    robot.stop()
    lm.hold()
    rm.hold()


leftColor = LightSensor(Port.S1)
rightColor = LightSensor(Port.S3)
myblocks = Generic_Myblocks(ev3, driver, leftMotor, rightMotor, leftColor, rightColor, inf)
myblocks.drive_mm(0, 1000, 100)