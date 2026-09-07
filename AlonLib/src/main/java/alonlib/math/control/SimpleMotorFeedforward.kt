package alonlib.math.control

import alonlib.math.PIDFGains
import kotlin.math.exp
import kotlin.math.sign

/**
 * Feedforward for a simple permanent-magnet DC motor: `u = ks*sign(v) + kv*v + ka*a`.
 *
 * Units are whatever [kv]/[ka] were tuned in -- radians for angular systems, meters for linear
 * ones.
 */
class SimpleMotorFeedforward(var gains: PIDFGains, val dt: Double = 0.020) {

	constructor(ks: Double = 0.0, kv: Double = 0.0, ka: Double = 0.0, kILimit: Double = 0.0) : this(PIDFGains(0.0, 0.0, 0.0, { 0.0 }, ks, kv, ka, kILimit))

	/** The feedforward for continuous control at [velocity]/[acceleration] (assumed 0 if omitted). */
	fun calculate(velocity: Double, acceleration: Double = 0.0) = gains.static * sign(velocity) + gains.velocity * velocity + gains.acceleration * acceleration

	/**
	 * The feedforward for exact discrete control stepping from [currentVelocity] to
	 * [nextVelocity] over [dt]. Inaccurate right where velocity crosses zero.
	 */
	fun calculateWithVelocities(currentVelocity: Double, nextVelocity: Double): Double {
		if (gains.acceleration < 1e-9) {
			return gains.static * sign(nextVelocity) + gains.velocity * nextVelocity
		}

		val a = -gains.velocity / gains.acceleration
		val b = 1.0 / gains.acceleration
		val ad = exp(a * dt)
		val bd = if (a > -1e-9) b * dt else 1.0 / a * (ad - 1.0) * b

		return gains.static * sign(currentVelocity) + 1.0 / bd * (nextVelocity - ad * currentVelocity)
	}

	/** The largest [velocity] achievable at [acceleration] without exceeding [maxVoltage]. */
	fun maxAchievableVelocity(maxVoltage: Double, acceleration: Double) = (maxVoltage - gains.static - acceleration * gains.acceleration) / gains.velocity

	/** The smallest (most negative) [velocity] achievable at [acceleration] without exceeding [maxVoltage]. */
	fun minAchievableVelocity(maxVoltage: Double, acceleration: Double) = (-maxVoltage + gains.static - acceleration * gains.acceleration) / gains.velocity

	/** The largest [acceleration] achievable at [velocity] without exceeding [maxVoltage]. */
	fun maxAchievableAcceleration(maxVoltage: Double, velocity: Double) = (maxVoltage - gains.static * sign(velocity) - velocity * gains.velocity) / gains.acceleration

	/** The smallest (most negative) [acceleration] achievable at [velocity] without exceeding [maxVoltage]. */
	fun minAchievableAcceleration(maxVoltage: Double, velocity: Double) = maxAchievableAcceleration(-maxVoltage, velocity)
}
