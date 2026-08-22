package org.firstinspires.ftc.teamcode.alonlib.math.filter

import org.firstinspires.ftc.teamcode.alonlib.math.clamp

/**
 * Limits how fast a value can change, in units/second -- e.g. a voltage or setpoint ramp. For
 * limiting a *position* instead of a rate, prefer [org.firstinspires.ftc.teamcode.alonlib.math.control.TrapezoidProfile].
 */
class SlewRateLimiter(
    private val positiveRateLimit: Double,
    private val negativeRateLimit: Double = -positiveRateLimit,
    initialValue: Double = 0.0,
) {
    var lastValue = initialValue
        private set

    private var lastTime = System.nanoTime() / 1e9

    fun calculate(input: Double): Double {
        val currentTime = System.nanoTime() / 1e9
        val elapsed = currentTime - lastTime
        lastValue += clamp(input - lastValue, negativeRateLimit * elapsed, positiveRateLimit * elapsed)
        lastTime = currentTime
        return lastValue
    }

    /** Resets to [value] immediately, bypassing the rate limit. */
    fun reset(value: Double) {
        lastValue = value
        lastTime = System.nanoTime() / 1e9
    }
}
