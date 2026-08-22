package org.firstinspires.ftc.teamcode.alonlib.math.control

import com.qualcomm.robotcore.hardware.PIDFCoefficients
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
class SquIDFController(kp: Double, ki: Double, kd: Double, kf: Double, sp: Double = 0.0, pv: Double = 0.0) :
        PIDFController(kp, ki, kd, kf, sp, pv) {

    constructor(coefficients: PIDFCoefficients) : this(coefficients.p, coefficients.i, coefficients.d, coefficients.f)

    override fun proportionalTerm(error: Double) = sign(error) * sqrt(abs(error))
}
