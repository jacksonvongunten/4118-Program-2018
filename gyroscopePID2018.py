from wpilib.drive import DifferentialDrive
import wpilib
import math
from networktables import NetworkTables



class GyroscopePID_Controller:

    def __init__(self, gyro, drive, goal):

        self.gyro = gyro
        self.drive = drive
        self.goal = goal
        self.lock = "locked"

        self.P = 0.7/self.goal
        self.I = 0
        self.D = 1/(4*(self.goal**2))

        self.integral = 0
        self.previousError = 0
        self.rcw = 0

        self.dash = NetworkTables.getTable('Data')



    def PID(self):
        error = self.goal-self.gyro.getAngle()
        self.integral += (error * 0.2)
        derivative = (error - self.previousError) / 0.02
        self.rcw = self.P*error + self.D*derivative
        self.previousError = error


    def execute(self):
        self.PID()
        if abs(self.rcw) < 0.1:
            self.lock = "released"
            return None
        else:
            if self.goal > 0:
                self.drive.arcadeDrive(0.0, self.rcw)
            else:
                self.drive.arcadeDrive(0.0, -self.rcw)
