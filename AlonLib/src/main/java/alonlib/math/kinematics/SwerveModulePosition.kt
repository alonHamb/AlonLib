package alonlib.math.kinematics

import alonlib.math.geometry.AngularPositon
import alonlib.math.interpolate
import alonlib.math.interpolation.Interpolatable
import kotlin.math.abs

/** Cumulative encoder distance + steering angle for one swerve module. */
class SwerveModulePosition(var distanceMeters: Double = 0.0, var angle: AngularPositon = AngularPositon.kZero) :
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
