package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d

/**
 * Dead-wheel odometry for a three-dead-wheel (left/right/horizontal) setup, with no gyro --
 * heading comes from the left/right delta and [trackWidth]; the horizontal (strafe) wheel is
 * corrected for [centerWheelOffset] (its distance from the robot's rotation center) so rotation
 * alone doesn't get misread as sideways drift.
 *
 * Pass [left]/[right]/[horizontal] to have [updatePose] (no args) pull live readings itself each
 * loop, or leave them unset and drive [update] directly with your own readings.
 */
class HolonomicOdometry(
    trackWidth: Double,
    private val centerWheelOffset: Double,
    initialPose: Pose2d = Pose2d(),
    private val left: (() -> Double)? = null,
    private val right: (() -> Double)? = null,
    private val horizontal: (() -> Double)? = null,
) : DeadWheelOdometryBase(initialPose, trackWidth) {

    private var previousAngle = initialPose.rotation
    private var prevLeftEncoder = 0.0
    private var prevRightEncoder = 0.0
    private var prevHorizontalEncoder = 0.0

    override fun updatePose(newPose: Pose2d) {
        previousAngle = newPose.rotation
        pose = newPose
        prevLeftEncoder = 0.0
        prevRightEncoder = 0.0
        prevHorizontalEncoder = 0.0
    }

    /** Pulls the latest readings from the [left]/[right]/[horizontal] lambdas passed to the constructor and updates [pose]. */
    override fun updatePose() {
        val left = left ?: return
        val right = right ?: return
        val horizontal = horizontal ?: return
        update(left(), right(), horizontal())
    }

    fun update(leftEncoderPos: Double, rightEncoderPos: Double, horizontalEncoderPos: Double): Pose2d {
        val deltaLeft = leftEncoderPos - prevLeftEncoder
        val deltaRight = rightEncoderPos - prevRightEncoder
        val deltaHorizontal = horizontalEncoderPos - prevHorizontalEncoder

        val angle = previousAngle + Rotation2d((deltaLeft - deltaRight) / trackWidth)

        prevLeftEncoder = leftEncoderPos
        prevRightEncoder = rightEncoderPos
        prevHorizontalEncoder = horizontalEncoderPos

        val dw = (angle - previousAngle).radians
        val dx = (deltaLeft + deltaRight) / 2.0
        val dy = deltaHorizontal - (centerWheelOffset * dw)

        val newPose = pose.exp(Twist2d(dx, dy, dw))

        previousAngle = angle
        pose = Pose2d(newPose.translation, angle)
        return pose
    }
}
