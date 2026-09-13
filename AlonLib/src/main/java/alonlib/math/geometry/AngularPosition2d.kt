package alonlib.math.geometry

import alonlib.math.interpolation.Interpolatable
import alonlib.math.mapRange
import alonlib.robotPrintError
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
class AngularPosition2d private constructor(val radians: Double, val cos: Double, val sin: Double) :
	Interpolatable<AngularPosition2d> {

	constructor(radians: Double = 0.0) : this(radians, cos(radians), sin(radians))

	/** Constructs a [AngularPosition2d] from the angle of the vector `(x, y)`, e.g. from a joystick. */
	constructor(x: Double, y: Double) : this(
		if (hypot(x, y) < 1e-9) 0.0 else atan2(y, x),
		if (hypot(x, y) < 1e-9) 1.0 else x / hypot(x, y),
		if (hypot(x, y) < 1e-9) 0.0 else y / hypot(x, y),
	)

	val degrees get() = Math.toDegrees(radians)
	val rotations get() = radians / (Math.PI * 2.0)
	val tan get() = sin / cos

	operator fun plus(other: AngularPosition2d) = rotateBy(other)
	operator fun minus(other: AngularPosition2d) = rotateBy(-other)
	operator fun unaryMinus() = AngularPosition2d(-radians, cos, -sin)
	operator fun times(other: AngularPosition2d) = AngularPosition2d(radians * other.radians)
	operator fun times(scalar: Number) = AngularPosition2d(radians * scalar.toDouble())
	operator fun div(other: AngularPosition2d) = AngularPosition2d(radians / other.radians)
	operator fun div(scalar: Number) = AngularPosition2d(radians / scalar.toDouble())
	fun coerceIn(min: AngularPosition2d, max: AngularPosition2d): AngularPosition2d {
		if (min.radians <= max.radians) {
			robotPrintError("min must be <= max")
		}
		return fromRadians(radians.coerceIn(min.radians, max.radians))
	}

	fun mapRange(value: AngularPosition2d, startMin: AngularPosition2d, startMax: AngularPosition2d, endMin: AngularPosition2d, endMax: AngularPosition2d): AngularPosition2d {
		return fromRadians(mapRange(value.radians, startMin.radians, startMax.radians, endMin.radians, endMax.radians))
	}

	/**
	 * Composes this rotation with [other] (i.e. rotates this by [other]).
	 *
	 * Routes through the `(x, y)` constructor rather than summing [radians] directly, so the
	 * result's [radians]/[degrees] stay normalized to (-180, 180] degrees
	 */
	fun rotateBy(other: AngularPosition2d) = AngularPosition2d(
		cos * other.cos - sin * other.sin,
		cos * other.sin + sin * other.cos,
	)

	override fun interpolate(endValue: AngularPosition2d, t: Double) = this + (endValue - this) * t

	override fun equals(other: Any?): Boolean {
		if (other !is AngularPosition2d) return false
		return abs(radians - other.radians) < 1e-9 ||
				(abs(cos - other.cos) < 1e-9 && abs(sin - other.sin) < 1e-9)
	}

	override fun hashCode() = radians.hashCode()

	override fun toString() = "Rotation2d(rads=$radians, deg=$degrees)"

	companion object {

		val kZero = AngularPosition2d(0.0)
		val kPi = AngularPosition2d(Math.PI)

		fun fromDegrees(degrees: Double) = AngularPosition2d(Math.toRadians(degrees))
		fun fromRadians(radians: Double) = AngularPosition2d(radians)
		fun fromRotations(rotations: Double) = AngularPosition2d(rotations * (Math.PI * 2.0))
	}
}
