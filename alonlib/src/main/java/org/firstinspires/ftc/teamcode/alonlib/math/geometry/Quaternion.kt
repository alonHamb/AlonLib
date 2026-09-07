package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * A quaternion -- backs [Rotation3d] rather than being used directly in most application code.
 *
 * Ported from WPILib's `Quaternion` (`org.wpilib.math.geometry.Quaternion`), including its
 * general (non-unit) [exp]/[log]/[pow] operators used by [Rotation3d.interpolate]'s slerp.
 */
class Quaternion(val w: Double = 1.0, val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0) {

    operator fun plus(other: Quaternion) = Quaternion(w + other.w, x + other.x, y + other.y, z + other.z)
    operator fun minus(other: Quaternion) = Quaternion(w - other.w, x - other.x, y - other.y, z - other.z)
    operator fun div(scalar: Double) = Quaternion(w / scalar, x / scalar, y / scalar, z / scalar)
    operator fun times(scalar: Double) = Quaternion(w * scalar, x * scalar, y * scalar, z * scalar)

    /** The Hamilton product of this quaternion with [other]. */
    operator fun times(other: Quaternion): Quaternion {
        val r1 = w
        val r2 = other.w

        val dot = x * other.x + y * other.y + z * other.z

        val crossX = y * other.z - other.y * z
        val crossY = other.x * z - x * other.z
        val crossZ = x * other.y - other.x * y

        return Quaternion(
            r1 * r2 - dot,
            r1 * other.x + r2 * x + crossX,
            r1 * other.y + r2 * y + crossY,
            r1 * other.z + r2 * z + crossZ,
        )
    }

    fun conjugate() = Quaternion(w, -x, -y, -z)

    fun dot(other: Quaternion) = w * other.w + x * other.x + y * other.y + z * other.z

    fun inverse(): Quaternion {
        val n = norm()
        return conjugate() / (n * n)
    }

    fun norm() = sqrt(dot(this))

    fun normalize(): Quaternion {
        val n = norm()
        return if (n == 0.0) Quaternion() else Quaternion(w / n, x / n, y / n, z / n)
    }

    /** `q^t`, via `exp(t * log(q))`. */
    fun pow(t: Double) = log().times(t).exp()

    /** The matrix exponential of this quaternion. Inverse of [log]. See `wpimath/docs/Quaternion.md`. */
    fun exp(): Quaternion {
        val scalar = exp(w)

        val axialMagnitude = sqrt(x * x + y * y + z * z)
        val cosine = cos(axialMagnitude)

        val axialScalar = if (axialMagnitude < 1e-9) {
            // Taylor series of sin(t)/t near t=0.
            val sq = axialMagnitude * axialMagnitude
            1.0 - sq / 6.0 + sq * sq / 120.0
        } else {
            sin(axialMagnitude) / axialMagnitude
        }

        return Quaternion(cosine * scalar, x * axialScalar * scalar, y * axialScalar * scalar, z * axialScalar * scalar)
    }

    /** The logarithm of this (possibly non-unit) quaternion. Inverse of [exp]. */
    fun log(): Quaternion {
        val n = norm()
        if (n == 0.0) return Quaternion(0.0, 0.0, 0.0, 0.0)

        val scalar = ln(n)
        val vNorm = sqrt(x * x + y * y + z * z)
        val sNorm = w / n

        if (abs(sNorm + 1.0) < 1e-9) {
            return Quaternion(scalar, -Math.PI, 0.0, 0.0)
        }

        val vScalar = when {
            vNorm < 1e-9 && w != 0.0 -> 1.0 / w - 1.0 / 3.0 * vNorm * vNorm / (w * w * w)
            vNorm == 0.0              -> 0.0
            else                       -> atan2(vNorm, w) / vNorm
        }

        return Quaternion(scalar, vScalar * x, vScalar * y, vScalar * z)
    }

    /** The axis-angle rotation vector (SO(3) log) this unit quaternion represents. */
    fun toRotationVector(): Translation3d {
        val n = sqrt(x * x + y * y + z * z)

        val coeff = if (n < 1e-9) {
            2.0 / w - 2.0 / 3.0 * n * n / (w * w * w)
        } else if (w < 0.0) {
            2.0 * atan2(-n, -w) / n
        } else {
            2.0 * atan2(n, w) / n
        }

        return Translation3d(coeff * x, coeff * y, coeff * z)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Quaternion) return false
        return abs(dot(other) - norm() * other.norm()) < 1e-9 && abs(norm() - other.norm()) < 1e-9
    }

    override fun hashCode() = arrayOf(w, x, y, z).contentHashCode()

    override fun toString() = "Quaternion($w, $x, $y, $z)"

    companion object {
        /** The exp operator of 𝖘𝖔(3): the quaternion representation of [rvec], an axis-angle rotation vector. */
        fun fromRotationVector(rvec: Translation3d): Quaternion {
            val theta = rvec.norm

            val cos = cos(theta / 2.0)
            val axialScalar = if (theta < 1e-9) {
                // Taylor series of sin(t/2)/t near t=0.
                1.0 / 2.0 - theta * theta / 48.0
            } else {
                sin(theta / 2.0) / theta
            }

            return Quaternion(cos, axialScalar * rvec.x, axialScalar * rvec.y, axialScalar * rvec.z)
        }
    }
}
