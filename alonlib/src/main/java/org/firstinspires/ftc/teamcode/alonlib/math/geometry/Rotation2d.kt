package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sign
import kotlin.math.sin

/**
 * A rotation in a 2D coordinate frame, represented internally by its cosine/sine rather than a
 * raw angle so composing rotations ([rotateBy]) is a cheap multiply instead of another
 * trig call.
 *
 * This is the internal, ported-from-WPILib geometry type -- distinct from RoadRunner's own
 * `Rotation2d` (bridged separately via `units/RoadRunnerConversions.kt`) and from
 * [org.firstinspires.ftc.teamcode.alonlib.units], though [org.firstinspires.ftc.teamcode.alonlib.units.Extensions.kt]-style
 * `Number.x` bridges for this type live in `units/WpilibConversions.kt`.
 */
class Rotation2d private constructor(val radians: Double, val cos: Double, val sin: Double) :
    Interpolatable<Rotation2d> {

    constructor(radians: Double = 0.0) : this(radians, cos(radians), sin(radians))

    /** Constructs a [Rotation2d] from the angle of the vector `(x, y)`, e.g. from a joystick. */
    constructor(x: Double, y: Double) : this(
        if (hypot(x, y) < 1e-9) 0.0 else atan2(y, x),
        if (hypot(x, y) < 1e-9) 1.0 else x / hypot(x, y),
        if (hypot(x, y) < 1e-9) 0.0 else y / hypot(x, y),
    )

    val degrees get() = Math.toDegrees(radians)
    val rotations get() = radians / (Math.PI * 2.0)
    val tan get() = sin / cos

    operator fun plus(other: Rotation2d) = rotateBy(other)
    operator fun minus(other: Rotation2d) = rotateBy(-other)
    operator fun unaryMinus() = Rotation2d(-radians, cos, -sin)
    operator fun times(scalar: Double) = Rotation2d(radians * scalar)
    operator fun div(scalar: Double) = Rotation2d(radians / scalar)
    fun coerceIn(min: Rotation2d, max: Rotation2d): Rotation2d {
        if (min.radians <= max.radians) {
            robotPrintError("min must be <= max")
        }
        return fromRadians(radians.coerceIn(min.radians, max.radians))
    }

    val normalizdDegrees
        get() = if (this.degrees.sign == -1.0) {
            this.degrees + 360.0
        } else {
            this.degrees
        }


    /**
     * Composes this rotation with [other] (i.e. rotates this by [other]).
     *
     * Routes through the `(x, y)` constructor rather than summing [radians] directly, so the
     * result's [radians]/[degrees] stay normalized to (-180, 180] degrees instead of drifting
     * unboundedly across repeated compositions (e.g. many [Odometry] ticks over a match).
     */
    fun rotateBy(other: Rotation2d) = Rotation2d(
        cos * other.cos - sin * other.sin,
        cos * other.sin + sin * other.cos,
    )

    override fun interpolate(endValue: Rotation2d, t: Double) = this + (endValue - this) * t

    override fun equals(other: Any?): Boolean {
        if (other !is Rotation2d) return false
        return kotlin.math.abs(radians - other.radians) < 1e-9 ||
                (kotlin.math.abs(cos - other.cos) < 1e-9 && kotlin.math.abs(sin - other.sin) < 1e-9)
    }

    override fun hashCode() = radians.hashCode()

    override fun toString() = "Rotation2d(rads=$radians, deg=$degrees)"

    companion object {
        val kZero = Rotation2d(0.0)
        val kPi = Rotation2d(Math.PI)

        fun fromDegrees(degrees: Double) = Rotation2d(Math.toRadians(degrees))
        fun fromRadians(radians: Double) = Rotation2d(radians)
        fun fromRotations(rotations: Double) = Rotation2d(rotations * (Math.PI * 2.0))
    }
}
