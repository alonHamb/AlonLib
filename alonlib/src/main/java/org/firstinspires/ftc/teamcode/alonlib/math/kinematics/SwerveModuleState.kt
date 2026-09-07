package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import kotlin.math.abs

/** The commanded/measured state (speed + steering angle) of one swerve module. */
class SwerveModuleState(var speedMetersPerSecond: Double = 0.0, var angle: Rotation2d = Rotation2d.kZero) :
        Comparable<SwerveModuleState> {

    /**
     * Minimizes the steering angle change to reach [angle] by potentially driving the wheel
     * backwards instead -- the module never needs to turn more than 90 degrees.
     */
    fun optimize(currentAngle: Rotation2d) {
        val delta = angle - currentAngle
        if (abs(delta.degrees) > 90.0) {
            speedMetersPerSecond *= -1.0
            angle = angle.rotateBy(Rotation2d.kPi)
        }
    }

    /**
     * Scales [speedMetersPerSecond] down by the cosine of the steering error, so a module that's
     * still turning towards its target angle doesn't drive perpendicular to where it should.
     */
    fun cosineScale(currentAngle: Rotation2d) {
        speedMetersPerSecond *= (angle - currentAngle).cos
    }

    override fun compareTo(other: SwerveModuleState) = speedMetersPerSecond.compareTo(other.speedMetersPerSecond)

    override fun equals(other: Any?): Boolean {
        if (other !is SwerveModuleState) return false
        return abs(speedMetersPerSecond - other.speedMetersPerSecond) < 1e-9 && angle == other.angle
    }

    override fun hashCode() = 31 * speedMetersPerSecond.hashCode() + angle.hashCode()

    override fun toString() = "SwerveModuleState(speed=$speedMetersPerSecond m/s, angle=$angle)"

    companion object {
        /** [SwerveModuleState.optimize], as a pure function returning the optimized copy instead of mutating in place. */
        fun optimize(desiredState: SwerveModuleState, currentAngle: Rotation2d): SwerveModuleState {
            val delta = desiredState.angle - currentAngle
            return if (abs(delta.degrees) > 90.0) {
                SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi))
            } else {
                SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle)
            }
        }
    }
}
