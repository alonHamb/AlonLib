package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

/**
 * A type that knows how to interpolate between two instances of itself.
 *
 * Implemented by [org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d] and friends so
 * they can be dropped straight into an [InterpolatingTreeMap]/trajectory sample without a separate
 * [Interpolator].
 */
interface Interpolatable<T> {
    /**
     * @returns the interpolation between this value and [endValue] at [t], where [t] = 0 returns
     * this value and [t] = 1 returns [endValue].
     */
    fun interpolate(endValue: T, t: Double): T
}
