import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
import encoderPID2018
import gyroscopePID2018
import math
import time



class Robot(wpilib.IterativeRobot):

	"""
	This block initializes everything and gets run once when the robot is turned on.
	If there is something that needs to be used throughout the entire code, then
	it should go here.
	"""
	def robotInit(self):
		self.lf_motor = wpilib.VictorSP(0)
		self.lb_motor = wpilib.VictorSP(1)
		self.left = wpilib.SpeedControllerGroup(self.lf_motor, self.lb_motor)

		self.rf_motor = wpilib.VictorSP(6)
		self.rb_motor = wpilib.VictorSP(7)
		self.right = wpilib.SpeedControllerGroup(self.rf_motor, self.rb_motor)

		self.roller = wpilib.VictorSP(2)

		self.l_encoder = wpilib.Encoder(0, 1)
		self.r_encoder = wpilib.Encoder(2, 3)

		self.gyro = wpilib.ADXRS450_Gyro(0)

		self.drive = DifferentialDrive(self.left, self.right)

		self.drive.setSafetyEnabled(False)

		self.controller = wpilib.XboxController(0)

		self.timer = wpilib.Timer()

		"""
		an array for all of the various "stages" inside of an autonomous program.
		This includes every stage (in our case, 5) to be executed sequentially.
		"""
		self.right_center_auto = []
		self.right_center_auto.append(encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, 2.8*(650.1875/math.pi)))
		self.right_center_auto.append(gyroscopePID2018.GyroscopePID_Controller(self.gyro, self.drive, 45))
		self.right_center_auto.append(encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, (7/math.sqrt(2))*(650.1875/math.pi)))
		self.right_center_auto.append(gyroscopePID2018.GyroscopePID_Controller(self.gyro, self.drive, -45))
		self.right_center_auto.append(encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, 1.2*(650.1875/math.pi)))

		self.left_center_auto = []
		self.left_center_auto.append(encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, 2.8*(650.1875/math.pi)))
		self.left_center_auto.append(gyroscopePID2018.GyroscopePID_Controller(self.gyro, self.drive, -45))
		self.left_center_auto.append(encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, (7/math.sqrt(2))*(650.1875/math.pi)))
		self.left_center_auto.append(gyroscopePID2018.GyroscopePID_Controller(self.gyro, self.drive, 45))
		self.left_center_auto.append(encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, 1.2*(650.1875/math.pi)))

		self.forward_auto = encoderPID2018.EncoderPID_Controller(self.l_encoder, self.r_encoder, self.drive, 11*(650.1875/math.pi))

		"""
		RoboRIO IP is always 10.XX.XX.2 where XX.XX is the team number
		"""
		NetworkTables.initialize(server="10.41.18.2")

		self.dash = NetworkTables.getTable('Data')

		"""
		Since we are going to determine which autonomous to run by using the NetworkTables
		value inside of "position", we initialize it to "none" until we update it.
		"""
		self.dash.putString("position", "none")

	"""
	This function resets everything to it's default values as if the robot was just
	rebooted
	"""
	def reset(self):
		self.drive.arcadeDrive(0.0, 0.0)
		self.gyro.reset()
		self.l_encoder.reset()
		self.r_encoder.reset()
		self.timer.reset()
		self.forward_data = None
		self.angle_data = None
		self.forward_iteration = 0
		self.angle_iteration = 0
		self.powers = None
		for stage in self.right_center_auto:
			stage.lock = "locked"
		for stage in self.left_center_auto:
			stage.lock = "locked"
		self.forward_auto.lock = "locked"

	"""
	run whenever teleop starts
	"""
	def teleopInit(self):
		self.reset()

	"""
	run periodically in teleop. In our case, just drive the robot with the controller
	"""
	def teleopPeriodic(self):
		forward = self.controller.getY(0)
		angle = self.controller.getX(1)
		self.drive.arcadeDrive(forward*0.65, angle*0.65)

		suck = self.controller.getTriggerAxis(0)
		out = -self.controller.getTriggerAxis(1)

		if (suck != 0):
			self.roller.set(suck)
		elif (out != 0):
			self.roller.set(out)
		else:
			self.roller.set(0)

	"""
	right-center autonomous execution thread
	"""
	def right_center(self):
		if self.right_center_auto[0].lock == "locked":
			self.right_center_auto[0].execute()

		if self.right_center_auto[0].lock == "released" and self.right_center_auto[1].lock == "locked":
			self.right_center_auto[1].execute()

		if self.right_center_auto[1].lock == "released":
			self.l_encoder.reset()
			self.r_encoder.reset()
			self.right_center_auto[1].lock = "released1"

		if self.right_center_auto[2].lock == "locked" and self.right_center_auto[1].lock == "released1":
			self.right_center_auto[2].execute()

		if self.right_center_auto[2].lock == "released":
			self.gyro.reset()
			self.right_center_auto[2].lock = "released1"

		if self.right_center_auto[3].lock == "locked" and self.right_center_auto[2].lock == "released1":
			self.right_center_auto[3].execute()

		if self.right_center_auto[3].lock == "released":
			self.l_encoder.reset()
			self.r_encoder.reset()
			self.right_center_auto[3].lock = "released1"

		if self.right_center_auto[4].lock == "locked" and self.right_center_auto[3].lock == "released1":
			self.roller.set(1.0)
			self.right_center_auto[4].execute()

		if self.right_center_auto[3].lock == "released":
			if self.timer.get() < 10:
				self.roller.set(1.0)
			else:
				self.roller.set(0.0)
			self.drive.arcadeDrive(0.0, 0.0)


	"""
	left-center autonomous execution thread
	"""
	def left_center(self):
		if self.left_center_auto[0].lock == "locked":
			self.left_center_auto[0].execute()

		if self.left_center_auto[0].lock == "released" and self.left_center_auto[1].lock == "locked":
			self.left_center_auto[1].execute()

		if self.left_center_auto[1].lock == "released":
			self.l_encoder.reset()
			self.r_encoder.reset()
			self.left_center_auto[1].lock = "released1"

		if self.left_center_auto[2].lock == "locked" and self.left_center_auto[1].lock == "released1":
			self.left_center_auto[2].execute()

		if self.left_center_auto[2].lock == "released":
			self.gyro.reset()
			self.left_center_auto[2].lock = "released1"

		if self.left_center_auto[3].lock == "locked" and self.left_center_auto[2].lock == "released1":
			self.left_center_auto[3].execute()

		if self.left_center_auto[3].lock == "released":
			self.l_encoder.reset()
			self.r_encoder.reset()
			self.left_center_auto[3].lock = "released1"

		if self.left_center_auto[4].lock == "locked" and self.left_center_auto[3].lock == "released1":
			self.roller.set(1.0)
			self.left_center_auto[4].execute()

		if self.left_center_auto[3].lock == "released":
			if self.timer.get() < 10:
				self.roller.set(1.0)
			else:
				self.roller.set(0.0)
			self.drive.arcadeDrive(0.0, 0.0)

	"""
	right-right autonomous execution thread
	"""
	def right_right(self):
		if self.forward_auto.lock == "released" or self.timer.get() > 5:
			self.roller.set(1.0)
		self.forward_auto.execute()

	"""
	left-left autonomous execution thread
	"""
	def left_left(self):
		if self.forward_auto.lock == "released" or self.timer.get() > 5:
			self.roller.set(1.0)
		self.forward_auto.execute()

	"""
	go-to-autoline autonomous execution thread
	"""
	def go_to_autoline(self):
		self.forward_auto.execute()

	"""
	run once at start of autonomous. Two possible options:
	1) Playback recorded data (PhaseAlpha)
	2) Follow a specific route
	"""
	def autonomousInit(self):
		self.reset()
		self.timer.start()
		self.game_data = wpilib.DriverStation.getInstance().getGameSpecificMessage()
		self.switch = self.game_data[0]
		self.position = self.dash.getString("position", "none")

	"""
	Either determine which autonomous to execute given the position in NetworkTables
	or simply playback a recording determined in "test" mode (see below)
	"""
	def autonomousPeriodic(self):
		if self.position == "solid":
			if self.timer.get() < 3:
				self.drive.arcadeDrive(-0.5, 0.0)
			else:
				self.drive.arcadeDrive(0.0, 0.0)

		if self.position == "none":
			self.drive.arcadeDrive(0.0, 0.0)

		if self.position == "center" and self.switch == "R":
			self.right_center()
		if self.position == "center" and self.switch == "L":
			self.left_center()

		if self.position == "right" and self.switch == "R":
			if self.timer.get() > 5:
				self.roller.set(1.0)
			self.right_right()
		if self.position == "left" and self.switch == "L":
			if self.timer.get() > 5:
				self.roller.set(1.0)
			self.left_left()

		if (self.position == "right" and self.switch == "L") or (self.position == "left" and self.switch == "R"):
			self.go_to_autoline()


"""
only execute as a script
"""
if __name__ == '__main__':
	wpilib.run(Robot)
