package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains
import org.firstinspires.ftc.teamcode.alonlib.math.clamp
import kotlin.math.abs
import kotlin.math.sign

/**
 * A PID controller with an added feedforward term proportional to the setpoint:
 * `u(t) = kP*e(t) + kI*∫e(t')dt' + kD*e'(t) + kF*r(t)`.
 *
 * Timing is wall-clock (`System.nanoTime()`), pass this
 * `calculate(measurement)` every loop
 */
open class PIDFController(var gains: PIDFGains, setPoint: Double = 0.0, current: Double = 0.0) {

	constructor(kP: Double = 0.0, kI: Double = 0.0, kd: Double = 0.0, kFF: (Double) -> Double = { 0.0 }, kS: Double = 0.0, kV: Double = 0.0, kA: Double = 0.0, kILimit: Double = 0.0) : this(PIDFGains(kP, kI, kd, kFF, kS, kV, kA, kILimit))

	/** Extra behavior for dealing with integral windup. */
	enum class IntegrationBehavior {

		/** No special behavior beyond clamping to [IntegrationControl]'s bounds. */
		NONE,

		/** Clears the accumulated integral once the controller reaches the setpoint within tolerance. */
		CLEAR_AT_SETPOINT,
	}

	/** Configures how [totalError] is bounded/decayed/cleared. See [IntegrationBehavior]. */
	class IntegrationControl(
		var integrationBehavior: IntegrationBehavior = IntegrationBehavior.NONE,
		var decayFactor: Double = 1.0,
		var minIntegral: Double = -1.0,
		var maxIntegral: Double = 1.0,
	) {

		fun setIntegrationBounds(min: Double, max: Double) {
			minIntegral = min
			maxIntegral = max
		}
	}

	var integrationControl = IntegrationControl()

	var totalError = 0.0
		protected set

	// --- setpoint / measurement ---

	var setPoint: Double = setPoint
		set(value) {
			field = value
			positionError = field - measuredValue
			velocityError = if (abs(period) > 1e-6) (positionError - prevError) / period else 0.0
		}

	var measuredValue: Double = current
		protected set

	var positionError = setPoint - current
		protected set

	var velocityError = 0.0
		protected set

	protected var prevError = 0.0
	protected var lastTimeStamp = 0.0
	var period = 0.0
		protected set

	// --- tolerance / output bounds ---

	var toleranceP = 0.05
		private set

	var toleranceV = Double.POSITIVE_INFINITY
		private set

	/** The minimum (magnitude of the) output enforced by [calculate] while not [inTolerance]. */
	var minOutput = 0.0
		set(value) {
			field = abs(value)
		}

	/** The maximum (magnitude of the) output enforced by [calculate] while not [inTolerance]. */
	var maxOutput = Double.POSITIVE_INFINITY

	/** A basic open-loop feedforward, sign-matched to the error, added on top of every [calculate] call. */
	var openF = 0.0

	fun setTolerance(positionTolerance: Double, velocityTolerance: Double = Double.POSITIVE_INFINITY) {
		toleranceP = positionTolerance
		toleranceV = velocityTolerance
	}

	fun inTolerance() = abs(positionError) < toleranceP && abs(velocityError) < toleranceV

	// --- gains ---

	fun setGains(kp: Double, ki: Double, kd: Double, kf: Double) {
		gains.proportional = kp
		gains.integral = ki
		gains.derivative = kd
		gains.feedForward = { kf }
	}

	fun clearTotalError() {
		totalError = 0.0
	}

	// --- calculation ---

	protected open fun calculateOutput(pv: Double): Double {
		prevError = positionError

		val now = System.nanoTime() / 1e9
		if (lastTimeStamp == 0.0) lastTimeStamp = now
		period = now - lastTimeStamp
		lastTimeStamp = now

		if (measuredValue != pv) measuredValue = pv
		positionError = setPoint - measuredValue

		velocityError = if (abs(period) > 1e-6) (positionError - prevError) / period else 0.0

		totalError += period * (setPoint - measuredValue)
		totalError = clamp(totalError, integrationControl.minIntegral, integrationControl.maxIntegral)
		if (sign(totalError) != sign(positionError)) {
			totalError *= integrationControl.decayFactor
		}
		if (inTolerance() && integrationControl.integrationBehavior == IntegrationBehavior.CLEAR_AT_SETPOINT) {
			clearTotalError()
		}

		return gains.proportional * proportionalTerm(positionError) + gains.integral * totalError + gains.derivative * velocityError + gains.feedForward(setPoint)
	}

	/**
	 * The value the kP gain is multiplied by, given the current [positionError] -- override to
	 * shape the proportional term (e.g. [SquIDFController] sign-preserving-square-roots it).
	 * Defaults to the error itself, i.e. a standard linear P term.
	 */
	protected open fun proportionalTerm(error: Double) = error * gains.proportional

	/** Calculates the next controller output for measurement [current]. */
	fun calculate(current: Double): Double {
		var output = calculateOutput(current)
		output += sign(positionError) * openF
		return if (inTolerance()) {
			output
		} else {
			clamp(abs(output), minOutput, maxOutput) * sign(output)
		}
	}

	/** Sets [setPoint] to [sp], then calculates the next controller output for measurement [pv]. */
	fun calculate(pv: Double, sp: Double): Double {
		setPoint = sp
		return calculate(pv)
	}

	/** Calculates the next controller output using the last-seen [measuredValue]. */
	fun calculate() = calculate(measuredValue)

	open fun reset() {
		prevError = 0.0
		lastTimeStamp = 0.0
		totalError = 0.0
	}
}
