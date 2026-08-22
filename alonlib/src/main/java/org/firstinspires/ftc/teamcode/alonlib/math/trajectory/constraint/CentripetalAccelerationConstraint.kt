package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Caps centripetal acceleration (`v^2 / r = v^2 * curvature`), slowing the robot through tight
 * turns so sharp-cornered trajectories stay trackable.
 */
class CentripetalAccelerationConstraint(private val maxCentripetalAccelerationMetersPerSecondSq: Double) : TrajectoryConstraint {

    override fun getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): Double {
        // ac = v^2 * k  =>  v = sqrt(ac / k)
        return sqrt(maxCentripetalAccelerationMetersPerSecondSq / abs(curvatureRadPerMeter))
    }

    override fun getMinMaxAccelerationMetersPerSecondSq(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double) =
        TrajectoryConstraint.MinMax()
}
