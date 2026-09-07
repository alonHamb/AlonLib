package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

/**
 * The inverse of [Interpolator]: given [startValue], [endValue] and a value [q] somewhere between
 * them, returns how far along that interpolation [q] sits, as a fraction in `[0, 1]`.
 *
 * Used by [InterpolatingTreeMap] to place a lookup key between its two bracketing entries.
 */
fun interface InverseInterpolator<T> {
    fun inverseInterpolate(startValue: T, endValue: T, q: T): Double

    companion object {
        /** An [InverseInterpolator] for plain [Double] values. */
        val forDouble = InverseInterpolator<Double> { startValue, endValue, q ->
            val totalRange = endValue - startValue
            if (totalRange <= 0.0) {
                0.0
            } else {
                val queryToStart = q - startValue
                if (queryToStart <= 0.0) 0.0 else (queryToStart / totalRange).coerceIn(0.0, 1.0)
            }
        }
    }
}
