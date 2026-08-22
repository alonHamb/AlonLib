package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d

/**
 * Tracks a robot's field pose over time from a gyro angle plus wheel encoder readings. Use one of
 * the drivetrain-specific subclasses ([DifferentialDriveOdometry], [MecanumDriveOdometry],
 * [SwerveDriveOdometry]) rather than this directly.
 *
 * The gyro angle is trusted over the kinematics-derived rotation (encoders alone can't
 * distinguish wheel slip from an actual turn), while translation comes from the kinematics.
 */
open class Odometry<WheelPositions>(
    private val kinematics: Kinematics<*, WheelPositions>,
    gyroAngle: Rotation2d,
    wheelPositions: WheelPositions,
    initialPose: Pose2d = Pose2d.kZero,
) {
    var pose = initialPose
        private set

    private var gyroOffset = pose.rotation - gyroAngle
    private var previousAngle = pose.rotation
    private var previousWheelPositions = wheelPositions

    /** Resets the tracked pose and the encoder/gyro baselines it's measured from. */
    fun resetPosition(gyroAngle: Rotation2d, wheelPositions: WheelPositions, pose: Pose2d) {
        this.pose = pose
        previousAngle = pose.rotation
        gyroOffset = pose.rotation - gyroAngle
        previousWheelPositions = wheelPositions
    }

    fun resetPose(pose: Pose2d) {
        gyroOffset += pose.rotation - this.pose.rotation
        this.pose = pose
        previousAngle = pose.rotation
    }

    fun resetTranslation(translation: Translation2d) {
        pose = Pose2d(translation, pose.rotation)
    }

    fun resetRotation(rotation: Rotation2d) {
        gyroOffset += rotation - pose.rotation
        pose = Pose2d(pose.translation, rotation)
        previousAngle = pose.rotation
    }

    /** Integrates the latest [gyroAngle]/[wheelPositions] reading into [pose] and returns it. */
    fun update(gyroAngle: Rotation2d, wheelPositions: WheelPositions): Pose2d {
        val angle = gyroAngle + gyroOffset

        val twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions)
        val correctedTwist = Twist2d(twist.dx, twist.dy, (angle - previousAngle).radians)

        val newPose = pose.exp(correctedTwist)

        previousWheelPositions = wheelPositions
        previousAngle = angle
        pose = Pose2d(newPose.translation, angle)

        return pose
    }
}
