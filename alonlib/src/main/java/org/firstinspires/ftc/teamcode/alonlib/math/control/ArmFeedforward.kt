package org.firstinspires.ftc.teamcode.alonlib.math.control

import kotlin.math.cos
import kotlin.math.sign

/**
 * Feedforward for an arm (a motor fighting gravity on a beam pivoted at an angle):
 * `u = ks*sign(v) + kg*cos(position) + kv*v + ka*a`.
 *
 * [position] is measured from horizontal -- 0 means the arm is parallel to the floor. Offset your
 * encoder if it doesn't already follow that convention.
 */
class ArmFeedforward(var ks: Double, var kg: Double, var kv: Double, var ka: Double = 0.0, val dt: Double = 0.020) {

    /** The feedforward for continuous control at [positionRadians]/[velocityRadPerSec]/[accelRadPerSecSquared] (accel assumed 0 if omitted). */
    fun calculate(positionRadians: Double, velocityRadPerSec: Double, accelRadPerSecSquared: Double = 0.0) =
        ks * sign(velocityRadPerSec) + kg * cos(positionRadians) + kv * velocityRadPerSec + ka * accelRadPerSecSquared

    /** The largest [velocity] achievable at [angle]/[acceleration] without exceeding [maxVoltage]. */
    fun maxAchievableVelocity(maxVoltage: Double, angle: Double, acceleration: Double) =
        (maxVoltage - ks - cos(angle) * kg - acceleration * ka) / kv

    /** The smallest (most negative) [velocity] achievable at [angle]/[acceleration] without exceeding [maxVoltage]. */
    fun minAchievableVelocity(maxVoltage: Double, angle: Double, acceleration: Double) =
        (-maxVoltage + ks - cos(angle) * kg - acceleration * ka) / kv

    /** The largest [acceleration] achievable at [angle]/[velocity] without exceeding [maxVoltage]. */
    fun maxAchievableAcceleration(maxVoltage: Double, angle: Double, velocity: Double) =
        (maxVoltage - ks * sign(velocity) - cos(angle) * kg - velocity * kv) / ka

    /** The smallest (most negative) [acceleration] achievable at [angle]/[velocity] without exceeding [maxVoltage]. */
    fun minAchievableAcceleration(maxVoltage: Double, angle: Double, velocity: Double) =
        maxAchievableAcceleration(-maxVoltage, angle, velocity)
}
