class PIDController:
  def __init__(self):
    self.integral = 0
    self.last_error = 0

  def adjust(error, gainP, gainI, gainD):
    self.integral = error + self.integral
    derivative = error - self.last_error
    return (error * gainP) + (self.integral * gainI) + (derivative * gainD)