package org.firstinspires.ftc.teamcode.alonlib.math.geometry

/**
 * A transformation (translation + rotation) that can be applied to a [Pose3d] via
 * [Pose3d.transformBy]/`+`. Applied intrinsically, relative to the starting pose's own frame.
 */
class Transform3d(val translation: Translation3d = Translation3d.kZero, val rotation: Rotation3d = Rotation3d.kZero) {

    constructor(x: Double, y: Double, z: Double, rotation: Rotation3d) : this(Translation3d(x, y, z), rotation)

    /** The transform that carries [initial] to [last]. */
    constructor(initial: Pose3d, last: Pose3d) : this(
        (last.translation - initial.translation).rotateBy(initial.rotation.inverse()),
        last.rotation.relativeTo(initial.rotation),
    )

    /** Constructs a 3D transform from a 2D transform in the X-Y plane. */
    constructor(transform: Transform2d) : this(Translation3d(transform.translation), Rotation3d(transform.rotation))

    val x get() = translation.x
    val y get() = translation.y
    val z get() = translation.z

    fun times(scalar: Double) = Transform3d(translation * scalar, rotation.times(scalar))
    fun div(scalar: Double) = times(1.0 / scalar)

    operator fun plus(other: Transform3d) = Transform3d(Pose3d.kZero, Pose3d.kZero.transformBy(this).transformBy(other))

    fun inverse() = Transform3d(translation.unaryMinus().rotateBy(rotation.inverse()), rotation.inverse())

    override fun equals(other: Any?): Boolean {
        if (other !is Transform3d) return false
        return translation == other.translation && rotation == other.rotation
    }

    override fun hashCode() = 31 * translation.hashCode() + rotation.hashCode()

    override fun toString() = "Transform3d($translation, $rotation)"

    companion object {
        val kZero = Transform3d()
    }
}
