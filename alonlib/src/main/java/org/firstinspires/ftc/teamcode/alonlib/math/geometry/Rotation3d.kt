package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * A rotation in 3D space, backed by a [Quaternion]. Unlike [Rotation2d], 3D rotations don't
 * commute -- see WPILib's `Rotation3d` class doc for the classic worked example of why order
 * matters.
 *
 * Ported from WPILib's `Rotation3d`. [rotateBy] applies extrinsically (around the global axes);
 * [relativeTo] applies intrinsically (from the other rotation's own perspective).
 */
class Rotation3d(q: Quaternion) : Interpolatable<Rotation3d> {

    val quaternion: Quaternion = q.normalize()

    constructor() : this(Quaternion())

    /**
     * Constructs a [Rotation3d] from extrinsic roll/pitch/yaw (in that order, around the fixed
     * global axes -- not the body frame).
     */
    constructor(roll: Double, pitch: Double, yaw: Double) : this(rollPitchYawToQuaternion(roll, pitch, yaw))

    /** Constructs a [Rotation3d] from an axis-angle rotation vector (axis direction times angle in radians). */
    constructor(rotationVector: Translation3d) : this(rotationVector, rotationVector.norm)

    /** Constructs a [Rotation3d] with the given (not-necessarily-normalized) [axis] and [angleRadians]. */
    constructor(axis: Translation3d, angleRadians: Double) : this(axisAngleToQuaternion(axis, angleRadians))

    /** Constructs a 3D rotation from a 2D rotation in the X-Y plane (i.e. pure yaw). */
    constructor(rotation2d: Rotation2d) : this(0.0, 0.0, rotation2d.radians)

    val x get() = extractRoll(quaternion)
    val y get() = extractPitch(quaternion)
    val z get() = extractYaw(quaternion)

    fun inverse() = Rotation3d(quaternion.inverse())
    fun times(scalar: Double) = kZero.interpolate(this, scalar)
    fun div(scalar: Double) = times(1.0 / scalar)

    /** Composes this rotation with [other], applied extrinsically (around the global axes). */
    fun rotateBy(other: Rotation3d) = Rotation3d(other.quaternion * quaternion)

    /** This rotation, re-expressed relative to [other]'s orientation (applied intrinsically). */
    fun relativeTo(other: Rotation3d) = Rotation3d(other.quaternion.inverse() * quaternion)

    /** Integrates constant body-frame angular rates over [dtSeconds] to project this rotation forward. */
    fun integrate(rollRate: Double, pitchRate: Double, yawRate: Double, dtSeconds: Double): Rotation3d {
        val w = Quaternion(0.0, rollRate, pitchRate, yawRate)
        return Rotation3d(quaternion * (w * (dtSeconds / 2.0)).exp())
    }

    /** The (not necessarily unit-length before normalization) axis of the axis-angle representation. */
    val axis: Translation3d get() {
        val n = sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return if (n == 0.0) Translation3d() else Translation3d(quaternion.x / n, quaternion.y / n, quaternion.z / n)
    }

    /** The angle, in radians, of the axis-angle representation. */
    val angle: Double get() {
        val n = sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return 2.0 * atan2(n, quaternion.w)
    }

    /** This rotation as a row-major 3x3 rotation matrix. */
    fun toMatrix(): Array<DoubleArray> {
        val w = quaternion.w
        val x = quaternion.x
        val y = quaternion.y
        val z = quaternion.z
        return arrayOf(
            doubleArrayOf(1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            doubleArrayOf(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            doubleArrayOf(2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        )
    }

    /** The axis-angle rotation vector representation (SO(3) log) of this rotation. */
    fun toVector() = quaternion.toRotationVector()

    /** This rotation, projected into the X-Y plane (its yaw component). */
    fun toRotation2d() = Rotation2d(z)

    override fun interpolate(endValue: Rotation3d, t: Double): Rotation3d {
        // slerp(q0, q1, t) = (q1 * q0⁻¹)^t * q0, negating the delta quaternion if needed for the
        // shortest path.
        val q0 = quaternion
        val q1 = endValue.quaternion
        var delta = q1 * q0.inverse()
        if (delta.w < 0.0) {
            delta = Quaternion(-delta.w, -delta.x, -delta.y, -delta.z)
        }
        return Rotation3d(delta.pow(t) * q0)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Rotation3d) return false
        return abs(abs(quaternion.dot(other.quaternion)) - quaternion.norm() * other.quaternion.norm()) < 1e-9
    }

    override fun hashCode() = quaternion.hashCode()

    override fun toString() = "Rotation3d($quaternion)"

    companion object {
        val kZero = Rotation3d()

        fun fromRadians(roll: Double, pitch: Double, yaw: Double) = Rotation3d(roll, pitch, yaw)
        fun fromDegrees(roll: Double, pitch: Double, yaw: Double) =
            Rotation3d(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw))

        /**
         * Constructs a [Rotation3d] from a row-major special-orthogonal 3x3 [rotationMatrix], via
         * Shepperd's method. Prints an error and returns [kZero] if [rotationMatrix] isn't special
         * orthogonal (i.e. isn't a valid rotation matrix), rather than throwing.
         */
        fun fromRotationMatrix(rotationMatrix: Array<DoubleArray>): Rotation3d {
            val r = rotationMatrix
            if (!isSpecialOrthogonal(r)) {
                robotPrintError("rotation matrix is not special orthogonal")
                return kZero
            }

            val trace = r[0][0] + r[1][1] + r[2][2]
            val w: Double
            val x: Double
            val y: Double
            val z: Double

            if (trace > 0.0) {
                val s = 0.5 / sqrt(trace + 1.0)
                w = 0.25 / s
                x = (r[2][1] - r[1][2]) * s
                y = (r[0][2] - r[2][0]) * s
                z = (r[1][0] - r[0][1]) * s
            } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
                val s = 2.0 * sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
                w = (r[2][1] - r[1][2]) / s
                x = 0.25 * s
                y = (r[0][1] + r[1][0]) / s
                z = (r[0][2] + r[2][0]) / s
            } else if (r[1][1] > r[2][2]) {
                val s = 2.0 * sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
                w = (r[0][2] - r[2][0]) / s
                x = (r[0][1] + r[1][0]) / s
                y = 0.25 * s
                z = (r[1][2] + r[2][1]) / s
            } else {
                val s = 2.0 * sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
                w = (r[1][0] - r[0][1]) / s
                x = (r[0][2] + r[2][0]) / s
                y = (r[1][2] + r[2][1]) / s
                z = 0.25 * s
            }

            return Rotation3d(Quaternion(w, x, y, z))
        }

        /** Constructs the [Rotation3d] that rotates [initial] onto [last] (both arbitrary, non-zero vectors). */
        fun fromVectorToVector(initial: Translation3d, last: Translation3d): Rotation3d {
            val dot = initial.dot(last)
            val normProduct = initial.norm * last.norm
            val dotNorm = dot / normProduct

            return when {
                dotNorm > 1.0 - 1e-9  -> kZero
                dotNorm < -1.0 + 1e-9 -> {
                    // Antiparallel: rotate 180deg around any axis orthogonal to `initial`.
                    val ax = abs(initial.x)
                    val ay = abs(initial.y)
                    val az = abs(initial.z)
                    val other = if (ax < ay) {
                        if (ax < az) Translation3d(1.0, 0.0, 0.0) else Translation3d(0.0, 0.0, 1.0)
                    } else {
                        if (ay < az) Translation3d(0.0, 1.0, 0.0) else Translation3d(0.0, 0.0, 1.0)
                    }
                    val axis = cross(initial, other)
                    val axisNorm = axis.norm
                    Rotation3d(Quaternion(0.0, axis.x / axisNorm, axis.y / axisNorm, axis.z / axisNorm))
                }
                else                    -> {
                    val axis = cross(initial, last)
                    Rotation3d(Quaternion(normProduct + dot, axis.x, axis.y, axis.z).normalize())
                }
            }
        }

        private fun cross(a: Translation3d, b: Translation3d) =
            Translation3d(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y)

        private fun rollPitchYawToQuaternion(roll: Double, pitch: Double, yaw: Double): Quaternion {
            val cr = cos(roll * 0.5)
            val sr = sin(roll * 0.5)
            val cp = cos(pitch * 0.5)
            val sp = sin(pitch * 0.5)
            val cy = cos(yaw * 0.5)
            val sy = sin(yaw * 0.5)

            return Quaternion(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            )
        }

        private fun axisAngleToQuaternion(axis: Translation3d, angleRadians: Double): Quaternion {
            val norm = axis.norm
            if (norm == 0.0) return Quaternion()

            val scale = sin(angleRadians / 2.0) / norm
            return Quaternion(cos(angleRadians / 2.0), axis.x * scale, axis.y * scale, axis.z * scale)
        }

        private fun extractRoll(q: Quaternion): Double {
            val cxcy = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            val sxcy = 2.0 * (q.w * q.x + q.y * q.z)
            return if (cxcy * cxcy + sxcy * sxcy > 1e-20) atan2(sxcy, cxcy) else 0.0
        }

        private fun extractPitch(q: Quaternion): Double {
            val ratio = 2.0 * (q.w * q.y - q.z * q.x)
            return if (abs(ratio) >= 1.0) sign(ratio) * (Math.PI / 2.0) else asin(ratio)
        }

        private fun extractYaw(q: Quaternion): Double {
            val cycz = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            val cysz = 2.0 * (q.w * q.z + q.x * q.y)
            return if (cycz * cycz + cysz * cysz > 1e-20) {
                atan2(cysz, cycz)
            } else {
                atan2(2.0 * q.w * q.z, q.w * q.w - q.z * q.z)
            }
        }

        private fun isSpecialOrthogonal(r: Array<DoubleArray>): Boolean {
            // RRᵀ ≈ I (orthogonal) and det(R) ≈ 1 (proper, not a reflection).
            for (i in 0..2) {
                for (j in 0..2) {
                    var dot = 0.0
                    for (k in 0..2) dot += r[i][k] * r[j][k]
                    val expected = if (i == j) 1.0 else 0.0
                    if (abs(dot - expected) > 1e-9) return false
                }
            }
            val det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                    r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                    r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
            return abs(det - 1.0) < 1e-9
        }
    }
}
