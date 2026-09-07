package org.firstinspires.ftc.teamcode.alonlib.math.control

import kotlin.math.exp
import kotlin.math.sign

/**
 * Feedforward for an elevator (a motor fighting gravity in a straight line):
 * `u = ks*sign(v) + kg + kv*v + ka*a`.
 */
class ElevatorFeedforward(var ks: Double, var kg: Double, var kv: Double, var ka: Double = 0.0, val dt: Double = 0.020) {

    /** The feedforward for continuous control at [velocity]/[acceleration] (assumed 0 if omitted). */
    fun calculate(velocity: Double, acceleration: Double = 0.0) = ks * sign(velocity) + kg + kv * velocity + ka * acceleration

    /**
     * The feedforward for exact discrete control stepping from [currentVelocity] to
     * [nextVelocity] over [dt]. Inaccurate right where velocity crosses zero.
     */
    fun calculateWithVelocities(currentVelocity: Double, nextVelocity: Double): Double {
        if (ka < 1e-9) {
            return ks * sign(nextVelocity) + kg + kv * nextVelocity
        }

        val a = -kv / ka
        val b = 1.0 / ka
        val ad = exp(a * dt)
        val bd = if (a > -1e-9) b * dt else 1.0 / a * (ad - 1.0) * b

        return kg + ks * sign(currentVelocity) + 1.0 / bd * (nextVelocity - ad * currentVelocity)
    }

    /** The largest [velocity] achievable at [acceleration] without exceeding [maxVoltage]. */
    fun maxAchievableVelocity(maxVoltage: Double, acceleration: Double) = (maxVoltage - ks - kg - acceleration * ka) / kv

    /** The smallest (most negative) [velocity] achievable at [acceleration] without exceeding [maxVoltage]. */
    fun minAchievableVelocity(maxVoltage: Double, acceleration: Double) = (-maxVoltage + ks - kg - acceleration * ka) / kv

    /** The largest [acceleration] achievable at [velocity] without exceeding [maxVoltage]. */
    fun maxAchievableAcceleration(maxVoltage: Double, velocity: Double) = (maxVoltage - ks * sign(velocity) - kg - velocity * kv) / ka

    /** The smallest (most negative) [acceleration] achievable at [velocity] without exceeding [maxVoltage]. */
    fun minAchievableAcceleration(maxVoltage: Double, velocity: Double) = maxAchievableAcceleration(-maxVoltage, velocity)
}
