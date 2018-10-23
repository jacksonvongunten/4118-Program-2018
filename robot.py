import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
import encoderPID2018
import gyroscopePID2018
import math
import time



class Robot(wpilib.IterativeRobot):

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

		NetworkTables.initialize(server="10.41.18.2")

		self.dash = NetworkTables.getTable('Data')

		self.dash.putString("position", "none")



	def reset(self):
		self.drive.arcadeDrive(0.0, 0.0)
		self.gyro.reset()
		self.l_encoder.reset()
		self.r_encoder.reset()
		self.timer.reset()
		for stage in self.right_center_auto:
			stage.lock = "locked"
		for stage in self.left_center_auto:
			stage.lock = "locked"
		self.forward_auto.lock = "locked"



	def teleopInit(self):
		self.reset()

	def teleopPeriodic(self):
		forward = self.controller.getY(0)
		angle = self.controller.getX(1)
		self.drive.arcadeDrive(forward*0.75, angle*0.75)

		suck = self.controller.getTriggerAxis(0)
		out = -self.controller.getTriggerAxis(1)

		if (suck != 0):
			self.roller.set(suck)
		elif (out != 0):
			self.roller.set(out)
		else:
			self.roller.set(0)


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

	def right_right(self):
		if self.forward_auto.lock == "released" or self.timer.get() > 5:
			self.roller.set(1.0)
		self.forward_auto.execute()


	def left_left(self):
		if self.forward_auto.lock == "released" or self.timer.get() > 5:
			self.roller.set(1.0)
		self.forward_auto.execute()

	def go_to_autoline(self):
		self.forward_auto.execute()

	def autonomousInit(self):
		self.reset()
		self.timer.start()
		self.game_data = wpilib.DriverStation.getInstance().getGameSpecificMessage()
		self.switch = self.game_data[0]
		self.position = self.dash.getString("position", "none")

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

	def disabledInit(self):
		self.reset()



if __name__ == '__main__':
	wpilib.run(Robot)
