package org.firstinspires.ftc.teamcode.alonlib.drives

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Vector2d
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import kotlin.math.sin

class HaMecanumDriveHa(
	var motors: Array<HaMotor>,
	var rightSideMultiplier: Double = -1.0
) : HaRobotDrive() {

	/**
	 * Sets up the constructor for the mecanum drive.
	 * Automatically inverts right side by default
	 *
	 * @param frontLeft  the front left motor
	 * @param frontRight the front right motor
	 * @param backLeft   the back left motor
	 * @param backRight  the back right motor
	 */
	constructor(frontLeft: HaMotor, frontRight: HaMotor, backLeft: HaMotor, backRight: HaMotor) : this(
		arrayOf(frontLeft, frontRight, backLeft, backRight), -1.0
	)

	/**
	 * Checks if the right side motors are inverted.
	 *
	 * @return true if the multiplier for the right side is equal to -1.
	 */
	fun isRightSideInverted(): Boolean {
		return rightSideMultiplier == -1.0
	}

	/**
	 * Sets the right side inversion factor to the specified boolean.
	 *
	 * @param isInverted If true, sets the right side multiplier to -1 or 1 if false.
	 */
	fun setRightSideInverted(isInverted: Boolean) {
		rightSideMultiplier = if (isInverted) -1.0 else 1.0
	}

	/**
	 * Stop the motors.
	 */
	override fun stop() {
		for (x in motors) {
			x.stop()
		}
	}

	/**
	 * Drives the robot from the perspective of the robot itself rather than that
	 * of the driver.
	 *
	 * @param strafeSpeed  the horizontal speed of the robot, derived from input
	 * @param forwardSpeed the vertical speed of the robot, derived from input
	 * @param turnSpeed    the turn speed of the robot, derived from input
	 */
	fun driveRobotCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double) {
		driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0)
	}

	/**
	 * Drives the robot from the perspective of the robot itself rather than that
	 * of the driver.
	 *
	 * @param strafeSpeed  the horizontal speed of the robot, derived from input
	 * @param forwardSpeed the vertical speed of the robot, derived from input
	 * @param turnSpeed    the turn speed of the robot, derived from input
	 * @param squareInputs Square joystick inputs for finer control
	 */
	fun driveRobotCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean) {
		var strafeSpeed = strafeSpeed
		var forwardSpeed = forwardSpeed
		var turnSpeed = turnSpeed
		strafeSpeed = if (squareInputs) clipRange(squareInput(strafeSpeed)) else clipRange(strafeSpeed)
		forwardSpeed = if (squareInputs) clipRange(squareInput(forwardSpeed)) else clipRange(forwardSpeed)
		turnSpeed = if (squareInputs) clipRange(squareInput(turnSpeed)) else clipRange(turnSpeed)

		driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed)
	}

	/**
	 * Drives the robot from the perspective of the driver. No matter the orientation of the
	 * robot, pushing forward on the drive stick will always drive the robot away
	 * from the driver.
	 *
	 * @param strafeSpeed  the horizontal speed of the robot, derived from input
	 * @param forwardSpeed the vertical speed of the robot, derived from input
	 * @param turnSpeed    the turn speed of the robot, derived from input
	 * @param gyroAngle    the heading of the robot, derived from the gyro
	 */
	fun driveFieldCentric(
		strafeSpeed: Double, forwardSpeed: Double,
		turnSpeed: Double, gyroAngle: Double
	) {
		var strafeSpeed = strafeSpeed
		var forwardSpeed = forwardSpeed
		var turnSpeed = turnSpeed
		strafeSpeed = clipRange(strafeSpeed)
		forwardSpeed = clipRange(forwardSpeed)
		turnSpeed = clipRange(turnSpeed)

		var input = Vector2d(strafeSpeed, forwardSpeed)
		input = input.rotateBy(-gyroAngle)

		val theta = input.angle()

		val wheelSpeeds = DoubleArray(4)
		wheelSpeeds[MotorType.FRONT_LEFT.value] = sin(theta + Math.PI / 4)
		wheelSpeeds[MotorType.FRONT_RIGHT.value] = sin(theta - Math.PI / 4)
		wheelSpeeds[MotorType.BACK_LEFT.value] = sin(theta - Math.PI / 4)
		wheelSpeeds[MotorType.BACK_RIGHT.value] = sin(theta + Math.PI / 4)

		normalize(wheelSpeeds, input.magnitude())

		wheelSpeeds[MotorType.FRONT_LEFT.value] += turnSpeed
		wheelSpeeds[MotorType.FRONT_RIGHT.value] -= turnSpeed
		wheelSpeeds[MotorType.BACK_LEFT.value] += turnSpeed
		wheelSpeeds[MotorType.BACK_RIGHT.value] -= turnSpeed

		normalize(wheelSpeeds)

		driveWithMotorPowers(
			wheelSpeeds[MotorType.FRONT_LEFT.value],
			wheelSpeeds[MotorType.FRONT_RIGHT.value],
			wheelSpeeds[MotorType.BACK_LEFT.value],
			wheelSpeeds[MotorType.BACK_RIGHT.value]
		)
	}

	/**
	 * Drives the robot from the perspective of the driver. No matter the orientation of the
	 * robot, pushing forward on the drive stick will always drive the robot away
	 * from the driver.
	 *
	 * @param xSpeed       the horizontal speed of the robot, derived from input
	 * @param ySpeed       the vertical speed of the robot, derived from input
	 * @param turnSpeed    the turn speed of the robot, derived from input
	 * @param gyroAngle    the heading of the robot, derived from the gyro
	 * @param squareInputs Square the value of the input to allow for finer control
	 */
	fun driveFieldCentric(xSpeed: Double, ySpeed: Double, turnSpeed: Double, gyroAngle: Double, squareInputs: Boolean) {
		var xSpeed = xSpeed
		var ySpeed = ySpeed
		var turnSpeed = turnSpeed
		xSpeed = if (squareInputs) clipRange(squareInput(xSpeed)) else clipRange(xSpeed)
		ySpeed = if (squareInputs) clipRange(squareInput(ySpeed)) else clipRange(ySpeed)
		turnSpeed = if (squareInputs) clipRange(squareInput(turnSpeed)) else clipRange(turnSpeed)

		driveFieldCentric(xSpeed, ySpeed, turnSpeed, gyroAngle)
	}

	/**
	 * Drives the motors directly with the specified motor powers.
	 *
	 * @param frontLeftSpeed    the speed of the front left motor
	 * @param frontRightSpeed   the speed of the front right motor
	 * @param backLeftSpeed     the speed of the back left motor
	 * @param backRightSpeed    the speed of the back right motor
	 */
	fun driveWithMotorPowers(
		frontLeftSpeed: Double, frontRightSpeed: Double,
		backLeftSpeed: Double, backRightSpeed: Double
	) {

		motors[MotorType.FRONT_LEFT.value]
			.percentOutput = (frontLeftSpeed * maxOutput).fraction
		motors[MotorType.FRONT_RIGHT.value]
			.percentOutput = (frontRightSpeed * rightSideMultiplier * maxOutput).fraction
		motors[MotorType.BACK_LEFT.value]
			.percentOutput = (backLeftSpeed * maxOutput).fraction
		motors[MotorType.BACK_RIGHT.value]
			.percentOutput = (backRightSpeed * rightSideMultiplier * maxOutput).fraction
	}

}
