package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.SwerveDriveKinematics
import kotlin.math.hypot

/** Caps trajectory velocity so no module of a swerve drivetrain exceeds [maxSpeedMetersPerSecond]. */
class SwerveDriveKinematicsConstraint(private val kinematics: SwerveDriveKinematics, private val maxSpeedMetersPerSecond: Double) :
        TrajectoryConstraint {

    override fun getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): Double {
        val xVelocity = velocityMetersPerSecond * pose.rotation.cos
        val yVelocity = velocityMetersPerSecond * pose.rotation.sin

        val chassisSpeeds = ChassisSpeeds(xVelocity, yVelocity, velocityMetersPerSecond * curvatureRadPerMeter)

        val moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMetersPerSecond)

        val normSpeeds = kinematics.toChassisSpeeds(moduleStates)
        return hypot(normSpeeds.vx, normSpeeds.vy)
    }

    override fun getMinMaxAccelerationMetersPerSecondSq(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double) =
        TrajectoryConstraint.MinMax()
}
