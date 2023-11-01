import math
import motor
import motor_pair
import runloop

from hub import port

class PIDController:
    def __init__(self, gain_p=0.7, gain_i=0.0, gain_d=1.2):
        # initial gain numbers (P: 0.7, I: 0, D: 1.2) from "Mastering LEGO Mindstorms"
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.integral = 0.0
        self.last_error = 0.0

    def adjust(self, error):
        self.integral = error + self.integral
        derivative = error - self.last_error
        return (error * self.gain_p) + (self.integral * self.gain_i) + (derivative * self.gain_d)

class HappyAccidents:
    def __init__(self):
        self.left_motor = hub.port.B
        self.right_motor = hub.port.F
        motor_pair.pair(motor_pair.PAIR_1, self.left_motor, self.right_motor)
        self.arm_motor = hub.port.D
        self.left_light = hub.port.A
        self.right_light = hub.port.E
        self.ultrasonic = hub.port.C
        self.wheel_circumference = math.pi * 57.0
        self.wheel_mm_degrees = 360 / self.wheel_circumference
        self.wheelbase_circumference = math.pi * 112.0
        hub.motion_sensor.set_yaw_face(hub.motion_sensor.FRONT)
 
    async def arm_left(self, duration, velocity=500, reset_arm=True):
        await motor.run_for_time(self.arm_motor, duration, velocity)
        if reset_arm:
            await self.arm_reset()

    async def arm_right(self, duration, velocity=500, reset_arm=True):
        await motor.run_for_time(self.arm_motor, duration, (0 - velocity))
        if reset_arm:
            await self.arm_reset()

    async def arm_reset(self, velocity=200):
        await motor.run_to_absolute_position(self.arm_motor, 45, velocity)

    def drive_mm_to_degrees(self, mm):
        return round(self.wheel_mm_degrees * mm)

    async def drive_mm(self, mm, velocity=200, steering=0):
        # motor_pair.move(motor_pair.PAIR_1, self.drive_mm_to_degrees(mm), steering=steering, velocity=velocity)
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, self.drive_mm_to_degrees(mm), velocity, velocity)

    def gyro_drive(self, mm, velocity=200, angle=0):
        self.reset()
        target_degrees = self.drive_mm_to_degrees(mm)
        pid_controller = PIDController()
        while motor.relative_position(self.right_motor) < target_degrees:
            new_angle = round(pid_controller.adjust(angle - hub.motion_sensor.tilt_angles()[0]))
            motor_pair.move(motor_pair.PAIR_1, new_angle, velocity=velocity)
        motor_pair.stop(motor_pair.PAIR_1)

    def pivot_degrees_to_motor_degrees(self, degrees):
        mm_to_pivot = self.wheelbase_circumference / 360.0 * float(degrees)
        motor_degrees_to_pivot = self.drive_mm_to_degrees(mm_to_pivot)
        return self.drive_mm_to_degrees(motor_degrees_to_pivot)

    async def pivot(self, degrees, velocity=100):
        left_velocity = velocity
        right_velocity = 0 - velocity
        if 0 > degrees:
            left_velocity = 0 - velocity
            right_velocity = velocity
            degrees = 0 - degrees
        motor_degrees = round(self.pivot_degrees_to_motor_degrees(degrees) / 2)
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, motor_degrees, left_velocity, right_velocity)

    def reset(self):
        motor.reset_relative_position(self.left_motor, 0)
        motor.reset_relative_position(self.right_motor, 0)
        hub.motion_sensor.reset_yaw(0)

async def main():
    # write your code here
    # motor.run_to_absolute_position(port.D, 0, 100, direction= motor.COUNTERCLOCKWISE)
    # motor_pair.pair(motor_pair.PAIR_1, port.B, port.F)
    # await motor_pair.move_for_time(motor_pair.PAIR_1, 1000, 0, velocity= 200)
    # motor.run_for_time(port.D, 500, -100)
    # motor.run_for_time(port.D, 500, 100)
    hub.light_matrix.write("M1")
    robot = HappyAccidents()

    await robot.arm_reset()
    # robot.gyro_drive(300)
    await robot.drive_mm(370)
    await robot.pivot(87)
    await robot.arm_left(2000, 200)
    await robot.drive_mm(250)
    await robot.pivot(-65)
    await robot.drive_mm(250)
    await robot.pivot(30)
    await robot.arm_left(2000, 200)
    await robot.arm_left(2000, 200)
    await robot.drive_mm(200)
    await robot.pivot(45)
    await robot.drive_mm(300)
    await robot.arm_left(3000, 200, False)
    await robot.arm_reset()

runloop.run(main())
