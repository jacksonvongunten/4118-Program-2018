from wpilib.drive import DifferentialDrive
import math

class EncoderPID_Controller:

	def __init__(self, l_encoder, r_encoder, drive, goal):
		self.l_encoder = l_encoder
		self.r_encoder = r_encoder
		self.drive = drive
		self.goal = goal
		self.lock = "locked"

		self.P = 0.7/self.goal
		self.I = 1/((self.goal**2)/6)
		self.D = 1/(5*(self.goal**2))

		self.integral = 0
		self.previousError = 0
		self.rcw = 0



	def averageEncoder(self):
		return (self.r_encoder.get() - self.l_encoder.get())/2

	def PID(self):
		error = 2*self.goal - self.averageEncoder()
		self.integral += (error * 0.2)
		derivative = (error - self.previousError) / 0.02
		self.rcw = self.P*error + self.I*self.integral + self.D*derivative
		self.previousError = error



	def execute(self):
		self.PID()
		if abs(self.rcw) < 0.1:
			self.lock = "released"
			return None
		else:
			self.drive.arcadeDrive(-self.rcw, 0.0)
