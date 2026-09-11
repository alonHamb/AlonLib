package alonlib.math.geometry

import alonlib.math.interpolation.Interpolatable
import alonlib.math.mapRange
import alonlib.robotPrintError
import alonlib.units.radians
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * A rotation in a 2D coordinate frame, represented internally by its cosine/sine rather than a
 * raw angle so composing rotations ([rotateBy]) is a cheap multiply instead of another
 * trig call.
 *
 * This is the internal, ported-from-WPILib geometry type.
 */
class AngularPositon private constructor(val radians: Double, val cos: Double, val sin: Double) :
	Interpolatable<AngularPositon> {

	constructor(radians: Double = 0.0) : this(radians, cos(radians), sin(radians))

	/** Constructs a [AngularPositon] from the angle of the vector `(x, y)`, e.g. from a joystick. */
	constructor(x: Double, y: Double) : this(
		if (hypot(x, y) < 1e-9) 0.0 else atan2(y, x),
		if (hypot(x, y) < 1e-9) 1.0 else x / hypot(x, y),
		if (hypot(x, y) < 1e-9) 0.0 else y / hypot(x, y),
	)

	val degrees get() = Math.toDegrees(radians)
	val rotations get() = radians / (Math.PI * 2.0)
	val tan get() = sin / cos

	operator fun plus(other: AngularPositon) = rotateBy(other)
	operator fun minus(other: AngularPositon) = rotateBy(-other)
	operator fun unaryMinus() = AngularPositon(-radians, cos, -sin)
	operator fun times(other: AngularPositon) = AngularPositon(radians * other.radians)
	operator fun times(scalar: Number) = AngularPositon(radians * scalar.toDouble())
	operator fun div(other: AngularPositon) = AngularPositon(radians / other.radians)
	operator fun div(scalar: Number) = AngularPositon(radians / scalar.toDouble())
	fun coerceIn(min: AngularPositon, max: AngularPositon): AngularPositon {
		if (min.radians <= max.radians) {
			robotPrintError("min must be <= max")
		}
		return fromRadians(radians.coerceIn(min.radians, max.radians))
	}

	fun mapRange(value: AngularPositon, startMin: AngularPositon, startMax: AngularPositon, endMin: AngularPositon, endMax: AngularPositon): AngularPositon {
		return mapRange(value.radians, startMin.radians, startMax.radians, endMin.radians, endMax.radians).radians
	}

	/**
	 * Composes this rotation with [other] (i.e. rotates this by [other]).
	 *
	 * Routes through the `(x, y)` constructor rather than summing [radians] directly, so the
	 * result's [radians]/[degrees] stay normalized to (-180, 180] degrees instead of drifting
	 * unboundedly across repeated compositions (e.g. many [Odometry] ticks over a match).
	 */
	fun rotateBy(other: AngularPositon) = AngularPositon(
		cos * other.cos - sin * other.sin,
		cos * other.sin + sin * other.cos,
	)

	override fun interpolate(endValue: AngularPositon, t: Double) = this + (endValue - this) * t

	override fun equals(other: Any?): Boolean {
		if (other !is AngularPositon) return false
		return abs(radians - other.radians) < 1e-9 ||
				(abs(cos - other.cos) < 1e-9 && abs(sin - other.sin) < 1e-9)
	}

	override fun hashCode() = radians.hashCode()

	override fun toString() = "Rotation2d(rads=$radians, deg=$degrees)"

	companion object {

		val kZero = AngularPositon(0.0)
		val kPi = AngularPositon(Math.PI)

		fun fromDegrees(degrees: Double) = AngularPositon(Math.toRadians(degrees))
		fun fromRadians(radians: Double) = AngularPositon(radians)
		fun fromRotations(rotations: Double) = AngularPositon(rotations * (Math.PI * 2.0))
	}
}
