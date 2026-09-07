package org.firstinspires.ftc.teamcode.alonlib.drives

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.units.fraction

/**
 * A two-side (tank) drivebase: [left]/[right] motors (each optionally an [HaMotor] with its own
 * followers) driven together per side. Invert a motor yourself before passing it in if needed; the
 * right side is inverted by default (see [autoInvert]/[setRightSideInverted]).
 */
class HaDifferentialDriveHa(private val left: HaMotor, private val right: HaMotor, autoInvert: Boolean = true) : HaRobotDrive() {

	private var rightSideMultiplier = if (autoInvert) -1.0 else 1.0

	val isRightSideInverted get() = rightSideMultiplier == -1.0

	fun setRightSideInverted(isInverted: Boolean) {
		rightSideMultiplier = if (isInverted) -1.0 else 1.0
	}

	override fun stop() {
		left.stop()
		right.stop()
	}

	/** [forwardSpeed] drives both sides equally; [turnSpeed] adds to the left and subtracts from the right. */
	fun arcadeDrive(forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
		val forward = clip(forwardSpeed, squareInputs)
		val turn = clip(turnSpeed, squareInputs)

		val wheelSpeeds = doubleArrayOf(forward + turn, forward - turn)
		normalize(wheelSpeeds)

		left.percentOutput = (maxOutput * wheelSpeeds[0]).fraction
		right.percentOutput = (rightSideMultiplier * maxOutput * wheelSpeeds[1]).fraction
	}

	/** [leftSpeed]/[rightSpeed] drive their respective side directly. */
	fun tankDrive(leftSpeed: Double, rightSpeed: Double, squareInputs: Boolean = false) {
		val leftClipped = clip(leftSpeed, squareInputs)
		val rightClipped = clip(rightSpeed, squareInputs)

		val wheelSpeeds = doubleArrayOf(leftClipped, rightClipped)
		normalize(wheelSpeeds)

		left.percentOutput = (wheelSpeeds[0] * maxOutput).fraction
		right.percentOutput = (wheelSpeeds[1] * rightSideMultiplier * maxOutput).fraction
	}

	private fun clip(value: Double, square: Boolean) = if (square) clipRange(squareInput(value)) else clipRange(value)
}
