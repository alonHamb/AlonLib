package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

/**
 * Interpolates between [startValue] and [endValue] by [t], for types that don't implement
 * [Interpolatable] themselves (e.g. [Double], or a third-party type).
 */
fun interface Interpolator<T> {
    fun interpolate(startValue: T, endValue: T, t: Double): T

    companion object {
        /** An [Interpolator] for plain [Double] values, backed by [org.firstinspires.ftc.teamcode.alonlib.math.interpolate]. */
        val forDouble = Interpolator<Double> { startValue, endValue, t ->
            org.firstinspires.ftc.teamcode.alonlib.math.interpolate(startValue, endValue, t)
        }

        /** An [Interpolator] for any [Interpolatable] type, delegating to its own [Interpolatable.interpolate]. */
        fun <T : Interpolatable<T>> forInterpolatable() = Interpolator<T> { startValue, endValue, t ->
            startValue.interpolate(endValue, t)
        }
    }
}
