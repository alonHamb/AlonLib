package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics

/** Caps trajectory velocity so neither side of a differential drivetrain exceeds [maxSpeedMetersPerSecond]. */
class DifferentialDriveKinematicsConstraint(private val kinematics: DifferentialDriveKinematics, private val maxSpeedMetersPerSecond: Double) :
        TrajectoryConstraint {

    override fun getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): Double {
        val chassisSpeeds = ChassisSpeeds(velocityMetersPerSecond, 0.0, velocityMetersPerSecond * curvatureRadPerMeter)

        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)
        wheelSpeeds.desaturate(maxSpeedMetersPerSecond)

        return kinematics.toChassisSpeeds(wheelSpeeds).vx
    }

    override fun getMinMaxAccelerationMetersPerSecondSq(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double) =
        TrajectoryConstraint.MinMax()
}
