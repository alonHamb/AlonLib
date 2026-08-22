package org.firstinspires.ftc.teamcode.alonlib.drives

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Vector2d
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import kotlin.math.sin

/** A four-wheel mecanum drivebase. See https://www.youtube.com/watch?v=8rhAkjViHEQ for the kinematics derivation. */
class MecanumDrive(
    private val frontLeft: HaMotor,
    private val frontRight: HaMotor,
    private val backLeft: HaMotor,
    private val backRight: HaMotor,
    autoInvert: Boolean = true,
) : RobotDrive() {

    private val motors = arrayOf(frontLeft, frontRight, backLeft, backRight)
    private var rightSideMultiplier = if (autoInvert) -1.0 else 1.0

    val isRightSideInverted get() = rightSideMultiplier == -1.0

    fun setRightSideInverted(isInverted: Boolean) {
        rightSideMultiplier = if (isInverted) -1.0 else 1.0
    }

    override fun stop() = motors.forEach { it.stop() }

    /** Robot-relative: forward always drives the way the robot's currently facing. */
    fun driveRobotCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
        val strafe = clip(strafeSpeed, squareInputs)
        val forward = clip(forwardSpeed, squareInputs)
        val turn = clip(turnSpeed, squareInputs)
        driveFieldCentric(strafe, forward, turn, 0.0)
    }

    /** Field-relative: forward always drives away from the driver, regardless of the robot's [headingRadians]. */
    fun driveFieldCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double, headingRadians: Double, squareInputs: Boolean = false) {
        val strafe = clip(strafeSpeed, squareInputs)
        val forward = clip(forwardSpeed, squareInputs)
        val turn = clip(turnSpeed, squareInputs)

        var input = Vector2d(strafe, forward)
        input = input.rotateBy(-headingRadians)
        val theta = input.angle()

        val wheelSpeeds = DoubleArray(4)
        wheelSpeeds[MotorType.FRONT_LEFT.value] = sin(theta + Math.PI / 4)
        wheelSpeeds[MotorType.FRONT_RIGHT.value] = sin(theta - Math.PI / 4)
        wheelSpeeds[MotorType.BACK_LEFT.value] = sin(theta - Math.PI / 4)
        wheelSpeeds[MotorType.BACK_RIGHT.value] = sin(theta + Math.PI / 4)

        normalize(wheelSpeeds, input.magnitude())

        wheelSpeeds[MotorType.FRONT_LEFT.value] += turn
        wheelSpeeds[MotorType.FRONT_RIGHT.value] -= turn
        wheelSpeeds[MotorType.BACK_LEFT.value] += turn
        wheelSpeeds[MotorType.BACK_RIGHT.value] -= turn

        normalize(wheelSpeeds)

        driveWithMotorPowers(
            wheelSpeeds[MotorType.FRONT_LEFT.value],
            wheelSpeeds[MotorType.FRONT_RIGHT.value],
            wheelSpeeds[MotorType.BACK_LEFT.value],
            wheelSpeeds[MotorType.BACK_RIGHT.value],
        )
    }

    fun driveWithMotorPowers(frontLeftSpeed: Double, frontRightSpeed: Double, backLeftSpeed: Double, backRightSpeed: Double) {
        frontLeft.percentOutput = (frontLeftSpeed * maxOutput).fraction
        frontRight.percentOutput = (frontRightSpeed * rightSideMultiplier * maxOutput).fraction
        backLeft.percentOutput = (backLeftSpeed * maxOutput).fraction
        backRight.percentOutput = (backRightSpeed * rightSideMultiplier * maxOutput).fraction
    }

    private fun clip(value: Double, square: Boolean) = if (square) clipRange(squareInput(value)) else clipRange(value)
}
