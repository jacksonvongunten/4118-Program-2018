import wpilib

class Robot(wpilib.IterativeRobot):
	def robotInit(self):
		self.motor = wpilib.VictorSP(0)

	def teleopInit(self):
		self.motor.set(0)

	def teleopPeriodic(self):
		self.motor.set(0.5)

if __name__ == '__main__':
	wpilib.run(Robot)
