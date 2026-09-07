package org.firstinspires.ftc.teamcode.alonlib.math.geometry

/**
 * A transformation (translation + rotation) that can be applied to a [Pose2d] via
 * [Pose2d.transformBy]/`+`. Unlike [Pose2d], a [Transform2d] is relative rather than absolute --
 * its translation is expressed in the *starting* pose's rotated frame, not the field frame.
 */
class Transform2d(val translation: Translation2d = Translation2d.kZero, val rotation: Rotation2d = Rotation2d.kZero) {

    constructor(x: Double, y: Double, rotation: Rotation2d) : this(Translation2d(x, y), rotation)

    /** The relative transform that carries [initial] to [last]. */
    constructor(initial: Pose2d, last: Pose2d) : this(
        (last.translation - initial.translation).rotateBy(-initial.rotation),
        last.rotation - initial.rotation,
    )

    val x get() = translation.x
    val y get() = translation.y

    fun inverse() = Transform2d((-translation).rotateBy(-rotation), -rotation)

    operator fun plus(other: Transform2d) = Transform2d(Pose2d.kZero, Pose2d.kZero.transformBy(this).transformBy(other))
    operator fun times(scalar: Double) = Transform2d(translation * scalar, rotation * scalar)
    operator fun div(scalar: Double) = times(1.0 / scalar)

    override fun equals(other: Any?): Boolean {
        if (other !is Transform2d) return false
        return translation == other.translation && rotation == other.rotation
    }

    override fun hashCode() = 31 * translation.hashCode() + rotation.hashCode()

    override fun toString() = "Transform2d($translation, $rotation)"
}
