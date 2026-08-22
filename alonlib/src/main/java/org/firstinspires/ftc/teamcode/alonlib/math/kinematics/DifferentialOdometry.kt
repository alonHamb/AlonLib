package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d

/**
 * Dead-wheel odometry for a two-encoder (left/right) setup, with no gyro -- heading is derived
 * purely from the left/right encoder delta and [trackWidth].
 *
 * Pass [left]/[right] to have [updatePose] (no args) pull live readings itself each loop, or
 * leave them unset and drive [updatePosition] directly with your own readings.
 */
class DifferentialOdometry(
    trackWidth: Double,
    initialPose: Pose2d = Pose2d(),
    private val left: (() -> Double)? = null,
    private val right: (() -> Double)? = null,
) : DeadWheelOdometryBase(initialPose, trackWidth) {

    private var previousAngle = initialPose.rotation
    private var prevLeftEncoder = 0.0
    private var prevRightEncoder = 0.0

    override fun updatePose(newPose: Pose2d) {
        previousAngle = newPose.rotation
        pose = newPose
        prevLeftEncoder = 0.0
        prevRightEncoder = 0.0
    }

    /** Pulls the latest readings from the [left]/[right] lambdas passed to the constructor and updates [pose]. */
    override fun updatePose() {
        val left = left ?: return
        val right = right ?: return
        updatePosition(left(), right())
    }

    fun updatePosition(leftEncoderPos: Double, rightEncoderPos: Double): Pose2d {
        val deltaLeft = leftEncoderPos - prevLeftEncoder
        val deltaRight = rightEncoderPos - prevRightEncoder

        prevLeftEncoder = leftEncoderPos
        prevRightEncoder = rightEncoderPos

        val dx = (deltaLeft + deltaRight) / 2.0

        val angle = previousAngle + Rotation2d((deltaLeft - deltaRight) / trackWidth)

        val newPose = pose.exp(Twist2d(dx, 0.0, (angle - previousAngle).radians))

        previousAngle = angle
        pose = Pose2d(newPose.translation, angle)
        return pose
    }
}
