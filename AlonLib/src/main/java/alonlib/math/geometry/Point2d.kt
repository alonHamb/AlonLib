package alonlib.math.geometry

import alonlib.math.interpolate
import alonlib.math.interpolation.Interpolatable
import kotlin.math.abs
import kotlin.math.hypot

/** A point in a 2D coordinate frame, in meters. */
class Point2d(val x: Double = 0.0, val y: Double = 0.0) : Interpolatable<Point2d> {

	/** Constructs a [Point2d] from polar coordinates: [distance] at [angle] from the origin. */
	constructor(distance: Double, angle: AngularPosition2d) : this(distance * angle.cos, distance * angle.sin)

	val norm get() = hypot(x, y)
	val angle get() = AngularPosition2d(x, y)

	fun getDistance(other: Point2d) = hypot(other.x - x, other.y - y)

	/** Rotates this translation around the origin by [other]. */
	fun rotateBy(other: AngularPosition2d) = Point2d(x * other.cos - y * other.sin, x * other.sin + y * other.cos)

	/** Rotates this translation around [other] by [rotation]. */
	fun rotateAround(other: Point2d, rotation: AngularPosition2d) = (this - other).rotateBy(rotation) + other

	operator fun plus(other: Point2d) = Point2d(x + other.x, y + other.y)
	operator fun minus(other: Point2d) = Point2d(x - other.x, y - other.y)
	operator fun unaryMinus() = Point2d(-x, -y)
	operator fun times(scalar: Double) = Point2d(x * scalar, y * scalar)
	operator fun times(other: Point2d) = Point2d(x * other.x, y * other.y)
	operator fun div(scalar: Double) = Point2d(x / scalar, y / scalar)
	operator fun div(other: Point2d) = Point2d(x / other.x, y / other.y)

	override fun interpolate(endValue: Point2d, t: Double) =
		Point2d(interpolate(x, endValue.x, t), interpolate(y, endValue.y, t))

	override fun equals(other: Any?): Boolean {
		if (other !is Point2d) return false
		return abs(x - other.x) < 1e-9 && abs(y - other.y) < 1e-9
	}

	override fun hashCode() = 31 * x.hashCode() + y.hashCode()

	override fun toString() = "Translation2d(x=$x, y=$y)"

	companion object {

		val kZero = Point2d(0.0, 0.0)
	}
}
