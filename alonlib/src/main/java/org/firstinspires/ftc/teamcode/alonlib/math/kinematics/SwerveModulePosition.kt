package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.interpolate
import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.Interpolatable
import kotlin.math.abs

/** Cumulative encoder distance + steering angle for one swerve module. */
class SwerveModulePosition(var distanceMeters: Double = 0.0, var angle: Rotation2d = Rotation2d.kZero) :
        Comparable<SwerveModulePosition>, Interpolatable<SwerveModulePosition> {

    fun copy() = SwerveModulePosition(distanceMeters, angle)

    override fun compareTo(other: SwerveModulePosition) = distanceMeters.compareTo(other.distanceMeters)

    override fun interpolate(endValue: SwerveModulePosition, t: Double) =
        SwerveModulePosition(interpolate(distanceMeters, endValue.distanceMeters, t), angle.interpolate(endValue.angle, t))

    override fun equals(other: Any?): Boolean {
        if (other !is SwerveModulePosition) return false
        return abs(distanceMeters - other.distanceMeters) < 1e-9 && angle == other.angle
    }

    override fun hashCode() = 31 * distanceMeters.hashCode() + angle.hashCode()

    override fun toString() = "SwerveModulePosition(distance=$distanceMeters m, angle=$angle)"
}
