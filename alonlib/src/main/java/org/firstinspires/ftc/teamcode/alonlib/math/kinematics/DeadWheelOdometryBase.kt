package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/**
 * Base for dead-wheel-only odometry ([DifferentialOdometry], [HolonomicOdometry]) that computes
 * heading purely from encoder deltas, with no gyro input.
 *
 * Distinct from [Odometry] (which *is* gyro-driven, matching WPILib's per-wheel-kinematics
 * odometry) -- this is SolversLib's simpler dead-wheel-pod-oriented design, kept under its own
 * name to avoid colliding with that one in this package.
 */
abstract class DeadWheelOdometryBase(initialPose: Pose2d, val trackWidth: Double = 18.0) {

    var pose = initialPose
        protected set

    /** Recomputes [pose] from the latest live sensor readings (see the constructor lambdas of subclasses). */
    abstract fun updatePose()

    /** Resets [pose] outright and clears the subclass's running encoder deltas. */
    abstract fun updatePose(newPose: Pose2d)

    /** Offsets [pose]'s heading by [byRadians] without moving its translation. */
    fun rotatePose(byRadians: Double) {
        pose = Pose2d(pose.translation, pose.rotation + Rotation2d(byRadians))
    }
}
