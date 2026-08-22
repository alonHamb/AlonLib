package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d

/** A user-defined velocity/acceleration limit applied while generating a trajectory. */
interface TrajectoryConstraint {

    /** The absolute maximum velocity at [pose]/[curvatureRadPerMeter], starting from [velocityMetersPerSecond]. */
    fun getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): Double

    /** The acceleration bounds at [pose]/[curvatureRadPerMeter]/[velocityMetersPerSecond]. */
    fun getMinMaxAccelerationMetersPerSecondSq(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): MinMax

    data class MinMax(
        val minAccelerationMetersPerSecondSq: Double = -Double.MAX_VALUE,
        val maxAccelerationMetersPerSecondSq: Double = Double.MAX_VALUE,
    )
}
