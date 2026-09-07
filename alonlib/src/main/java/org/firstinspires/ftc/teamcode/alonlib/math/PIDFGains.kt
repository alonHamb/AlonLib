package org.firstinspires.ftc.teamcode.alonlib.math

/**
 * Contains the following gains:
 * - [kP] Proportional gain.
 * - [kI] Integral gain.
 * - [kD] Derivative gain.
 * - [kFF] Feed Forward gain calculation function.
 * - [kS] Static gain.
 * - [kV] Velocity gain.
 * - [kA] Acceleration gain.
 * - [kILimit] If the absolute error is above IZone, the integral accumulator is cleared
 * (making it ineffective). Motor controllers have this feature, but WPILib don't.
 **/
class PIDFGains @JvmOverloads constructor(
	kP: Number = 0.0,
	kI: Number = 0.0,
	kD: Number = 0.0,
	kFF: (Double) -> Double = { 0.0 },
	kS: Number = 0.0,
	kV: Number = 0.0,
	kA: Number = 0.0,
	kILimit: Number = 0.0,
) {

	var proportional = kP.toDouble()
	var integral = kI.toDouble()
	var derivative = kD.toDouble()
	var feedForward: (Double) -> Double = kFF
	val static = kS.toDouble()
	val velocity = kV.toDouble()
	val acceleration = kA.toDouble()

	override fun toString(): String {
		return "(kP: $proportional ,kI: $integral ,Kd: $derivative ,kFF: $feedForward ,kS:$static ,kV: $velocity ,kA:$acceleration )"
	}

	companion object {

		val emptyGains = PIDFGains(0, 0, 0, { 0.0 }, 0, 0, 0)
	}
}
