package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable

/** A robot/object pose (position + orientation) in 3D space, in meters/radians. */
class Pose3d(val translation: Translation3d = Translation3d.kZero, val rotation: Rotation3d = Rotation3d.kZero) :
        Interpolatable<Pose3d> {

    constructor(x: Double, y: Double, z: Double, rotation: Rotation3d) : this(Translation3d(x, y, z), rotation)

    /** Constructs a 3D pose from a 2D pose in the X-Y plane (z = 0, pure yaw). */
    constructor(pose: Pose2d) : this(Translation3d(pose.x, pose.y, 0.0), Rotation3d(0.0, 0.0, pose.rotation.radians))

    val x get() = translation.x
    val y get() = translation.y
    val z get() = translation.z

    operator fun plus(other: Transform3d) = transformBy(other)
    operator fun minus(other: Pose3d): Transform3d {
        val relative = relativeTo(other)
        return Transform3d(relative.translation, relative.rotation)
    }

    fun times(scalar: Double) = Pose3d(translation * scalar, rotation.times(scalar))
    fun div(scalar: Double) = times(1.0 / scalar)

    /** Rotates this pose's translation and rotation both around the origin, extrinsically, by [other]. */
    fun rotateBy(other: Rotation3d) = Pose3d(translation.rotateBy(other), rotation.rotateBy(other))

    /** Applies [other] (a relative/intrinsic transform, expressed in this pose's own frame) to this pose. */
    fun transformBy(other: Transform3d) =
        Pose3d(translation + other.translation.rotateBy(rotation), other.rotation.rotateBy(rotation))

    /** @returns this pose expressed relative to [other] instead of the field/origin frame. */
    fun relativeTo(other: Pose3d): Pose3d {
        val transform = Transform3d(other, this)
        return Pose3d(transform.translation, transform.rotation)
    }

    /** Rotates this pose around [point] (in the global frame) by [rotation]. */
    fun rotateAround(point: Translation3d, rotation: Rotation3d) =
        Pose3d(translation.rotateAround(point, rotation), this.rotation.rotateBy(rotation))

    /** This pose projected into the X-Y plane. */
    fun toPose2d() = Pose2d(translation.toTranslation2d(), rotation.toRotation2d())

    fun nearest(poses: Collection<Pose3d>) =
        poses.minWithOrNull(
            compareBy<Pose3d> { translation.getDistance(it.translation) }
                .thenBy { rotation.relativeTo(it.rotation).angle }
        )

    /**
     * Interpolates translation and rotation independently (lerp + slerp) -- see [Twist3d]'s doc
     * for why this isn't the constant-curvature twist-based interpolation [Pose2d.interpolate] uses.
     */
    override fun interpolate(endValue: Pose3d, t: Double): Pose3d {
        if (t <= 0.0) return this
        if (t >= 1.0) return endValue
        return Pose3d(translation.interpolate(endValue.translation, t), rotation.interpolate(endValue.rotation, t))
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Pose3d) return false
        return translation == other.translation && rotation == other.rotation
    }

    override fun hashCode() = 31 * translation.hashCode() + rotation.hashCode()

    override fun toString() = "Pose3d($translation, $rotation)"

    companion object {
        val kZero = Pose3d()
    }
}
