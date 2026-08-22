package org.firstinspires.ftc.teamcode.alonlib.drives

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.Motor

/**
 * A two-side (tank) drivebase: [left]/[right] motors (or [org.firstinspires.ftc.teamcode.alonlib.hardware.motors.MotorGroup]s)
 * driven together per side. Invert a motor yourself before passing it in if needed; the right
 * side is inverted by default (see [autoInvert]/[setRightSideInverted]).
 */
class DifferentialDrive(private val left: Motor, private val right: Motor, autoInvert: Boolean = true) : RobotDrive() {

    private var rightSideMultiplier = if (autoInvert) -1.0 else 1.0

    val isRightSideInverted get() = rightSideMultiplier == -1.0

    fun setRightSideInverted(isInverted: Boolean) {
        rightSideMultiplier = if (isInverted) -1.0 else 1.0
    }

    override fun stop() {
        left.stopMotor()
        right.stopMotor()
    }

    /** [forwardSpeed] drives both sides equally; [turnSpeed] adds to the left and subtracts from the right. */
    fun arcadeDrive(forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
        val forward = clip(forwardSpeed, squareInputs)
        val turn = clip(turnSpeed, squareInputs)

        val wheelSpeeds = doubleArrayOf(forward + turn, forward - turn)
        normalize(wheelSpeeds)

        left.set(maxOutput * wheelSpeeds[0])
        right.set(rightSideMultiplier * maxOutput * wheelSpeeds[1])
    }

    /** [leftSpeed]/[rightSpeed] drive their respective side directly. */
    fun tankDrive(leftSpeed: Double, rightSpeed: Double, squareInputs: Boolean = false) {
        val leftClipped = clip(leftSpeed, squareInputs)
        val rightClipped = clip(rightSpeed, squareInputs)

        val wheelSpeeds = doubleArrayOf(leftClipped, rightClipped)
        normalize(wheelSpeeds)

        left.set(wheelSpeeds[0] * maxOutput)
        right.set(wheelSpeeds[1] * rightSideMultiplier * maxOutput)
    }

    private fun clip(value: Double, square: Boolean) = if (square) clipRange(squareInput(value)) else clipRange(value)
}
