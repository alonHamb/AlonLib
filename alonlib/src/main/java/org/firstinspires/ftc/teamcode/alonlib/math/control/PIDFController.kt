package org.firstinspires.ftc.teamcode.alonlib.math.control

import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.alonlib.math.clamp
import kotlin.math.abs
import kotlin.math.sign

/**
 * A PID controller with an added feedforward term proportional to the setpoint:
 * `u(t) = kP*e(t) + kI*∫e(t')dt' + kD*e'(t) + kF*r(t)`.
 *
 * Timing is wall-clock (`System.nanoTime()`), matching SolversLib's `PIDFController` -- pass this
 * `calculate(measurement)` every loop and it figures out `dt` itself, rather than taking an
 * explicit period like WPILib's `PIDController` does.
 */
open class PIDFController(kp: Double, ki: Double, kd: Double, kf: Double, sp: Double = 0.0, pv: Double = 0.0) {

    constructor(coefficients: PIDFCoefficients) : this(coefficients.p, coefficients.i, coefficients.d, coefficients.f)

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

    var p = kp
    var i = ki
    var d = kd
    var f = kf

    var totalError = 0.0
        protected set

    // --- setpoint / measurement ---

    var setPoint: Double = sp
        set(value) {
            field = value
            positionError = field - measuredValue
            velocityError = if (abs(period) > 1e-6) (positionError - prevError) / period else 0.0
        }

    var measuredValue: Double = pv
        protected set

    var positionError = sp - pv
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

    /** The minimum (magnitude of the) output enforced by [calculate] while not [atSetPoint]. */
    var minOutput = 0.0
        set(value) {
            field = abs(value)
        }

    /** The maximum (magnitude of the) output enforced by [calculate] while not [atSetPoint]. */
    var maxOutput = Double.POSITIVE_INFINITY

    /** A basic open-loop feedforward, sign-matched to the error, added on top of every [calculate] call. */
    var openF = 0.0

    fun setTolerance(positionTolerance: Double, velocityTolerance: Double = Double.POSITIVE_INFINITY) {
        toleranceP = positionTolerance
        toleranceV = velocityTolerance
    }

    fun atSetPoint() = abs(positionError) < toleranceP && abs(velocityError) < toleranceV

    // --- gains ---

    fun setPIDF(kp: Double, ki: Double, kd: Double, kf: Double) {
        p = kp
        i = ki
        d = kd
        f = kf
    }

    fun setCoefficients(coefficients: PIDFCoefficients) = setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f)

    val coefficients get() = doubleArrayOf(p, i, d, f)

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
        if (atSetPoint() && integrationControl.integrationBehavior == IntegrationBehavior.CLEAR_AT_SETPOINT) {
            clearTotalError()
        }

        return p * proportionalTerm(positionError) + i * totalError + d * velocityError + f * setPoint
    }

    /**
     * The value the [p] gain is multiplied by, given the current [positionError] -- override to
     * shape the proportional term (e.g. [SquIDFController] sign-preserving-square-roots it).
     * Defaults to the error itself, i.e. a standard linear P term.
     */
    protected open fun proportionalTerm(error: Double) = error

    /** Calculates the next controller output for measurement [pv]. */
    fun calculate(pv: Double): Double {
        var output = calculateOutput(pv)
        output += sign(positionError) * openF
        return if (atSetPoint()) {
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
