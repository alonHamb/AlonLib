package org.firstinspires.ftc.teamcode.alonlib.drives

import kotlin.math.abs

/** Shared plumbing for [org.firstinspires.ftc.teamcode.alonlib.drives]' drivebase classes: input clipping/squaring and wheel-speed normalization. */
abstract class RobotDrive {

    /** A motor's position within a drivebase's motor array -- indices assume 2-4 motors, per the concrete drivebase. */
    enum class MotorType(val value: Int) {
        BACK_LEFT(2),
        BACK_RIGHT(3),
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        LEFT(0),
        RIGHT(1),
        SLIDE(2),
    }

    protected var rangeMin = DEFAULT_RANGE_MIN
    protected var rangeMax = DEFAULT_RANGE_MAX
    protected var maxOutput = DEFAULT_MAX_SPEED

    fun setMaxSpeed(maxOutput: Double) {
        this.maxOutput = maxOutput
    }

    fun getMaxSpeed() = maxOutput

    /** The clip range drive inputs are clamped to (default `[-1, 1]`). */
    fun setRange(min: Double, max: Double) {
        rangeMin = min
        rangeMax = max
    }

    fun clipRange(value: Double) = value.coerceIn(rangeMin, rangeMax)

    abstract fun stop()

    /** Scales [wheelSpeeds] (in place) so the largest-magnitude one has magnitude [magnitude]. */
    protected fun normalize(wheelSpeeds: DoubleArray, magnitude: Double) {
        val maxMagnitude = wheelSpeeds.maxOf { abs(it) }
        for (i in wheelSpeeds.indices) wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude * magnitude
    }

    /** Scales [wheelSpeeds] (in place) down to `[-1, 1]` if any exceeds it in magnitude, preserving their ratios. */
    protected fun normalize(wheelSpeeds: DoubleArray) {
        val maxMagnitude = wheelSpeeds.maxOf { abs(it) }
        if (maxMagnitude > 1) {
            for (i in wheelSpeeds.indices) wheelSpeeds[i] /= maxMagnitude
        }
    }

    /** Squares [input]'s magnitude while preserving its sign -- gives finer control near zero on joystick inputs. */
    protected fun squareInput(input: Double) = input * abs(input)

    companion object {
        const val DEFAULT_RANGE_MIN = -1.0
        const val DEFAULT_RANGE_MAX = 1.0
        const val DEFAULT_MAX_SPEED = 1.0
    }
}
