package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.firstinspires.ftc.teamcode.alonlib.math.interpolate
import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable
import kotlin.math.sqrt

/** A translation (point) in 3D space, in meters. */
class Translation3d(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0) : Interpolatable<Translation3d> {

    /**
     * Constructs a [Translation3d] from polar coordinates: [distance] at [angle] from the origin.
     * Equivalent to rotating `(distance, 0, 0)` by [angle] -- inlined here (rather than delegating
     * to [rotateBy]) since constructor delegation can't reuse an intermediate value.
     */
    constructor(distance: Double, angle: Rotation3d) : this(
        distance * (1.0 - 2.0 * (angle.quaternion.y * angle.quaternion.y + angle.quaternion.z * angle.quaternion.z)),
        distance * 2.0 * (angle.quaternion.x * angle.quaternion.y + angle.quaternion.w * angle.quaternion.z),
        distance * 2.0 * (angle.quaternion.x * angle.quaternion.z - angle.quaternion.w * angle.quaternion.y),
    )

    /** Constructs a 3D translation from a 2D translation in the X-Y plane (z = 0). */
    constructor(translation: Translation2d) : this(translation.x, translation.y, 0.0)

    val norm get() = sqrt(x * x + y * y + z * z)
    val squaredNorm get() = x * x + y * y + z * z

    fun getDistance(other: Translation3d): Double {
        val dx = other.x - x
        val dy = other.y - y
        val dz = other.z - z
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    fun getSquaredDistance(other: Translation3d): Double {
        val dx = other.x - x
        val dy = other.y - y
        val dz = other.z - z
        return dx * dx + dy * dy + dz * dz
    }

    /** Rotates this translation around the origin by [other] (via the quaternion sandwich product). */
    fun rotateBy(other: Rotation3d): Translation3d {
        val p = Quaternion(0.0, x, y, z)
        val qPrime = other.quaternion * p * other.quaternion.inverse()
        return Translation3d(qPrime.x, qPrime.y, qPrime.z)
    }

    /** Rotates this translation around [other] by [rotation]. */
    fun rotateAround(other: Translation3d, rotation: Rotation3d) = (this - other).rotateBy(rotation) + other

    fun dot(other: Translation3d) = x * other.x + y * other.y + z * other.z

    fun cross(other: Translation3d) =
        Translation3d(y * other.z - other.y * z, z * other.x - other.z * x, x * other.y - other.x * y)

    /** This translation projected into the X-Y plane. */
    fun toTranslation2d() = Translation2d(x, y)

    operator fun plus(other: Translation3d) = Translation3d(x + other.x, y + other.y, z + other.z)
    operator fun minus(other: Translation3d) = Translation3d(x - other.x, y - other.y, z - other.z)
    operator fun unaryMinus() = Translation3d(-x, -y, -z)
    operator fun times(scalar: Double) = Translation3d(x * scalar, y * scalar, z * scalar)
    operator fun div(scalar: Double) = Translation3d(x / scalar, y / scalar, z / scalar)

    fun nearest(translations: Collection<Translation3d>) = translations.minBy { getDistance(it) }

    override fun interpolate(endValue: Translation3d, t: Double) =
        Translation3d(interpolate(x, endValue.x, t), interpolate(y, endValue.y, t), interpolate(z, endValue.z, t))

    override fun equals(other: Any?): Boolean {
        if (other !is Translation3d) return false
        return kotlin.math.abs(x - other.x) < 1e-9 && kotlin.math.abs(y - other.y) < 1e-9 && kotlin.math.abs(z - other.z) < 1e-9
    }

    override fun hashCode() = arrayOf(x, y, z).contentHashCode()

    override fun toString() = "Translation3d(x=$x, y=$y, z=$z)"

    companion object {
        val kZero = Translation3d()
    }
}
