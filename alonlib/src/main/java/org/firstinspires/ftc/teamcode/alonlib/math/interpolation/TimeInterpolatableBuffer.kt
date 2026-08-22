package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

import java.util.TreeMap

/**
 * A rolling window of the last [historySizeSeconds] worth of [T] samples, keyed by timestamp --
 * for estimating a past value (e.g. a robot pose) at an arbitrary time within that window, as
 * [org.firstinspires.ftc.teamcode.alonlib.math.estimator.PoseEstimator] needs to replay odometry
 * against a latency-compensated vision measurement.
 */
class TimeInterpolatableBuffer<T>(private val interpolator: Interpolator<T>, private val historySizeSeconds: Double) {

    /** The raw backing map, sorted by timestamp -- exposed for replaying samples in order. */
    val internalBuffer = TreeMap<Double, T>()

    /** Records [sample] at [timeSeconds], evicting anything older than [historySizeSeconds]. */
    fun addSample(timeSeconds: Double, sample: T) {
        while (internalBuffer.isNotEmpty()) {
            val oldest = internalBuffer.firstKey()
            if (timeSeconds - oldest >= historySizeSeconds) internalBuffer.remove(oldest) else break
        }
        internalBuffer[timeSeconds] = sample
    }

    fun clear() = internalBuffer.clear()

    /** The sample at [timeSeconds] (exact or interpolated between its neighbors), or null if the buffer is empty. */
    fun getSample(timeSeconds: Double): T? {
        if (internalBuffer.isEmpty()) return null

        internalBuffer[timeSeconds]?.let { return it }

        val top = internalBuffer.ceilingEntry(timeSeconds)
        val bottom = internalBuffer.floorEntry(timeSeconds)

        return when {
            top == null && bottom == null -> null
            top == null -> bottom!!.value
            bottom == null -> top.value
            else -> interpolator.interpolate(bottom.value, top.value, (timeSeconds - bottom.key) / (top.key - bottom.key))
        }
    }

    companion object {
        fun <T : Interpolatable<T>> createBuffer(historySizeSeconds: Double) =
            TimeInterpolatableBuffer(Interpolator.forInterpolatable<T>(), historySizeSeconds)
    }
}
