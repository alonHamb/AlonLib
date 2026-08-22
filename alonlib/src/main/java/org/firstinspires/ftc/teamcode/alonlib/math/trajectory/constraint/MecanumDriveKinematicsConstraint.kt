package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveKinematics
import kotlin.math.hypot

/** Caps trajectory velocity so no wheel of a mecanum drivetrain exceeds [maxSpeedMetersPerSecond]. */
class MecanumDriveKinematicsConstraint(private val kinematics: MecanumDriveKinematics, private val maxSpeedMetersPerSecond: Double) :
        TrajectoryConstraint {

    override fun getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): Double {
        val xVelocity = velocityMetersPerSecond * pose.rotation.cos
        val yVelocity = velocityMetersPerSecond * pose.rotation.sin

        val chassisSpeeds = ChassisSpeeds(xVelocity, yVelocity, velocityMetersPerSecond * curvatureRadPerMeter)

        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)
        wheelSpeeds.desaturate(maxSpeedMetersPerSecond)

        val normSpeeds = kinematics.toChassisSpeeds(wheelSpeeds)
        return hypot(normSpeeds.vx, normSpeeds.vy)
    }

    override fun getMinMaxAccelerationMetersPerSecondSq(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double) =
        TrajectoryConstraint.MinMax()
}
