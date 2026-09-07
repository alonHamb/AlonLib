package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d

/**
 * Wraps odometry updates as a [SubsystemBase] so [pose] refreshes every scheduler tick without a
 * command needing to poll it explicitly.
 *
 * Takes plain lambdas rather than an [org.firstinspires.ftc.teamcode.alonlib.math.kinematics.Odometry]
 * instance directly, since that class's `update(gyroAngle, wheelPositions)` needs fresh sensor
 * readings supplied by the caller each tick -- pass `{ odometry.update(gyro.heading, wheelPositions) }`.
 */
class OdometrySubsystem(private val updatePose: () -> Pose2d, private val currentPose: () -> Pose2d) : SubsystemBase() {

    val pose: Pose2d get() = currentPose()

    override fun periodic() {
        updatePose()
    }
}
