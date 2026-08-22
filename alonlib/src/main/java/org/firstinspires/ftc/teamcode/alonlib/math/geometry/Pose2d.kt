package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/** A robot pose (position + heading) in a 2D coordinate frame, in meters/radians. */
class Pose2d(val translation: Translation2d = Translation2d.kZero, val rotation: Rotation2d = Rotation2d.kZero) :
        Interpolatable<Pose2d> {

    constructor(x: Double, y: Double, rotation: Rotation2d) : this(Translation2d(x, y), rotation)

    val x get() = translation.x
    val y get() = translation.y

    /** Rotates this pose's translation and rotation both around the origin by [other]. */
    fun rotateBy(other: Rotation2d) = Pose2d(translation.rotateBy(other), rotation.rotateBy(other))

    /** Applies [other] (a relative transform, expressed in this pose's rotated frame) to this pose. */
    fun transformBy(other: Transform2d) =
        Pose2d(translation + other.translation.rotateBy(rotation), rotation + other.rotation)

    operator fun plus(other: Transform2d) = transformBy(other)
    operator fun minus(other: Pose2d) = Transform2d(other, this)
    operator fun times(scalar: Double) = Pose2d(translation * scalar, rotation * scalar)
    operator fun div(scalar: Double) = times(1.0 / scalar)

    /** @returns this pose expressed relative to [other] instead of the field/origin frame. */
    fun relativeTo(other: Pose2d): Pose2d {
        val transform = Transform2d(other, this)
        return Pose2d(transform.translation, transform.rotation)
    }

    /**
     * Integrates a constant-curvature [twist] forward from this pose, e.g. one odometry tick's
     * worth of wheel motion. See [log] for the inverse.
     */
    fun exp(twist: Twist2d): Pose2d {
        val dx = twist.dx
        val dy = twist.dy
        val dtheta = twist.dtheta

        val sinTheta = sin(dtheta)
        val cosTheta = cos(dtheta)

        val s: Double
        val c: Double
        if (abs(dtheta) < 1e-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
            c = 0.5 * dtheta
        } else {
            s = sinTheta / dtheta
            c = (1 - cosTheta) / dtheta
        }

        val transform = Transform2d(Translation2d(dx * s - dy * c, dx * c + dy * s), Rotation2d(cosTheta, sinTheta))
        return this + transform
    }

    /** @returns the [Twist2d] that [exp] would integrate from this pose to reach [end]. Inverse of [exp]. */
    fun log(end: Pose2d): Twist2d {
        val transform = end.relativeTo(this)
        val dtheta = transform.rotation.radians
        val halfDtheta = dtheta / 2.0

        val cosMinusOne = transform.rotation.cos - 1.0

        val halfThetaByTanOfHalfDtheta = if (abs(cosMinusOne) < 1e-9) {
            1.0 - 1.0 / 12.0 * dtheta * dtheta
        } else {
            -(halfDtheta * transform.rotation.sin) / cosMinusOne
        }

        val translationPart = transform.translation
            .rotateBy(Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
            .times(hypot(halfThetaByTanOfHalfDtheta, halfDtheta))

        return Twist2d(translationPart.x, translationPart.y, dtheta)
    }

    override fun interpolate(endValue: Pose2d, t: Double): Pose2d {
        if (t <= 0.0) return this
        if (t >= 1.0) return endValue
        val twist = log(endValue)
        return exp(Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t))
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Pose2d) return false
        return translation == other.translation && rotation == other.rotation
    }

    override fun hashCode() = 31 * translation.hashCode() + rotation.hashCode()

    override fun toString() = "Pose2d($translation, $rotation)"

    companion object {
        val kZero = Pose2d()
    }
}
