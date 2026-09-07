package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.firstinspires.ftc.teamcode.alonlib.math.interpolate
import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable
import kotlin.math.hypot

/** A translation (point) in a 2D coordinate frame, in meters. */
class Translation2d(val x: Double = 0.0, val y: Double = 0.0) : Interpolatable<Translation2d> {

    /** Constructs a [Translation2d] from polar coordinates: [distance] at [angle] from the origin. */
    constructor(distance: Double, angle: Rotation2d) : this(distance * angle.cos, distance * angle.sin)

    val norm get() = hypot(x, y)
    val angle get() = Rotation2d(x, y)

    fun getDistance(other: Translation2d) = hypot(other.x - x, other.y - y)

    /** Rotates this translation around the origin by [other]. */
    fun rotateBy(other: Rotation2d) = Translation2d(x * other.cos - y * other.sin, x * other.sin + y * other.cos)

    /** Rotates this translation around [other] by [rotation]. */
    fun rotateAround(other: Translation2d, rotation: Rotation2d) = (this - other).rotateBy(rotation) + other

    operator fun plus(other: Translation2d) = Translation2d(x + other.x, y + other.y)
    operator fun minus(other: Translation2d) = Translation2d(x - other.x, y - other.y)
    operator fun unaryMinus() = Translation2d(-x, -y)
    operator fun times(scalar: Double) = Translation2d(x * scalar, y * scalar)
    operator fun div(scalar: Double) = Translation2d(x / scalar, y / scalar)

    override fun interpolate(endValue: Translation2d, t: Double) =
        Translation2d(interpolate(x, endValue.x, t), interpolate(y, endValue.y, t))

    override fun equals(other: Any?): Boolean {
        if (other !is Translation2d) return false
        return kotlin.math.abs(x - other.x) < 1e-9 && kotlin.math.abs(y - other.y) < 1e-9
    }

    override fun hashCode() = 31 * x.hashCode() + y.hashCode()

    override fun toString() = "Translation2d(x=$x, y=$y)"

    companion object {
        val kZero = Translation2d(0.0, 0.0)
    }
}
