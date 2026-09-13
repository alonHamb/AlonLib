package alonlib.math.geometry

import alonlib.math.interpolate
import alonlib.math.interpolation.Interpolatable
import kotlin.math.abs
import kotlin.math.sqrt

/** A point in 3D space, in meters. */
class Point3d(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0) : Interpolatable<Point3d> {

	/**
	 * Constructs a [Point3d] from polar coordinates: [distance] at [angle] from the origin.
	 * Equivalent to rotating `(distance, 0, 0)` by [angle] -- inlined here (rather than delegating
	 * to [rotateBy]) since constructor delegation can't reuse an intermediate value.
	 */
	constructor(distance: Double, angle: AngularPosition3d) : this(
		distance * (1.0 - 2.0 * (angle.quaternion.y * angle.quaternion.y + angle.quaternion.z * angle.quaternion.z)),
		distance * 2.0 * (angle.quaternion.x * angle.quaternion.y + angle.quaternion.w * angle.quaternion.z),
		distance * 2.0 * (angle.quaternion.x * angle.quaternion.z - angle.quaternion.w * angle.quaternion.y),
	)

	/** Constructs a 3D translation from a 2D translation in the X-Y plane (z = 0). */
	constructor(translation: Point2d) : this(translation.x, translation.y, 0.0)

	val norm get() = sqrt(x * x + y * y + z * z)
	val squaredNorm get() = x * x + y * y + z * z

	fun getDistance(other: Point3d): Double {
		val dx = other.x - x
		val dy = other.y - y
		val dz = other.z - z
		return sqrt(dx * dx + dy * dy + dz * dz)
	}

	fun getSquaredDistance(other: Point3d): Double {
		val dx = other.x - x
		val dy = other.y - y
		val dz = other.z - z
		return dx * dx + dy * dy + dz * dz
	}

	/** Rotates this translation around the origin by [other] (via the quaternion sandwich product). */
	fun rotateBy(other: AngularPosition3d): Point3d {
		val p = Quaternion(0.0, x, y, z)
		val qPrime = other.quaternion * p * other.quaternion.inverse()
		return Point3d(qPrime.x, qPrime.y, qPrime.z)
	}

	/** Rotates this translation around [other] by [rotation]. */
	fun rotateAround(other: Point3d, rotation: AngularPosition3d) = (this - other).rotateBy(rotation) + other

	fun dot(other: Point3d) = x * other.x + y * other.y + z * other.z

	fun cross(other: Point3d) =
		Point3d(y * other.z - other.y * z, z * other.x - other.z * x, x * other.y - other.x * y)

	/** This translation projected into the X-Y plane. */
	fun toTranslation2d() = Point2d(x, y)

	operator fun plus(other: Point3d) = Point3d(x + other.x, y + other.y, z + other.z)
	operator fun minus(other: Point3d) = Point3d(x - other.x, y - other.y, z - other.z)
	operator fun unaryMinus() = Point3d(-x, -y, -z)
	operator fun times(scalar: Double) = Point3d(x * scalar, y * scalar, z * scalar)
	operator fun times(other: Point3d) = Point3d(x * other.x, y * other.y, z * other.z)
	operator fun div(scalar: Double) = Point3d(x / scalar, y / scalar, z / scalar)
	operator fun div(other: Point3d) = Point3d(x / other.x, y / other.y, z / other.z)

	fun nearest(translations: Collection<Point3d>) = translations.minBy { getDistance(it) }

	override fun interpolate(endValue: Point3d, t: Double) =
		Point3d(interpolate(x, endValue.x, t), interpolate(y, endValue.y, t), interpolate(z, endValue.z, t))

	override fun equals(other: Any?): Boolean {
		if (other !is Point3d) return false
		return abs(x - other.x) < 1e-9 && abs(y - other.y) < 1e-9 && abs(z - other.z) < 1e-9
	}

	override fun hashCode() = arrayOf(x, y, z).contentHashCode()

	override fun toString() = "Translation3d(x=$x, y=$y, z=$z)"

	companion object {

		val kZero = Point3d()
	}
}
