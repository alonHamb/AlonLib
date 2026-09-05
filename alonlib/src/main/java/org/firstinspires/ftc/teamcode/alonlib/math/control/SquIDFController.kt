package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * A [PIDFController] whose proportional term is sign-preserving-square-rooted:
 * `u(t) = kP*sign(e)*sqrt(|e|) + kI*∫e(t')dt' + kD*e'(t) + kF*r(t)`.
 *
 * Useful when a linear P term is too aggressive far from the setpoint (a large error produces a
 * disproportionately large correction) -- the square root flattens that out while keeping the
 * same direction and zero-crossing.
 */
class SquIDFController(gains: PIDFGains, setpoint: Double, current: Double) :
	PIDFController(gains) {

	constructor(kp: Double, ki: Double, kd: Double, kf: (Double) -> Double, sp: Double, pv: Double) : this(PIDFGains(kp, ki, kd, kf), sp, pv)

	override fun proportionalTerm(error: Double) = sign(error) * sqrt(abs(error))
}
