package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d
import org.firstinspires.ftc.teamcode.alonlib.math.interpolation.TimeInterpolatableBuffer
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.Odometry
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import java.util.TreeMap
import kotlin.math.sqrt

/**
 * Wraps an [Odometry] to fuse latency-compensated global measurements (e.g. from vision) with
 * wheel/gyro odometry. A drop-in replacement for [Odometry] -- behaves identically to it as long
 * as [addVisionMeasurement] is never called. Use one of the drivetrain-specific subclasses
 * ([DifferentialDrivePoseEstimator], [MecanumDrivePoseEstimator], [SwerveDrivePoseEstimator])
 * rather than this directly.
 */
open class PoseEstimator<WheelPositions>(
    private val odometry: Odometry<WheelPositions>,
    stateStdDevs: Matrix,
    visionMeasurementStdDevs: Matrix,
) {
    private val q = Matrix.vector(
        stateStdDevs[0, 0] * stateStdDevs[0, 0],
        stateStdDevs[1, 0] * stateStdDevs[1, 0],
        stateStdDevs[2, 0] * stateStdDevs[2, 0],
    )
    private val visionK = Matrix(3, 3)

    private val odometryPoseBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(BUFFER_DURATION_SECONDS)
    private val visionUpdates = TreeMap<Double, VisionUpdate>()

    var estimatedPosition = odometry.pose
        private set

    init {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs)
    }

    /** Changes how much [addVisionMeasurement] trusts future global measurements (e.g. as distance to a vision target changes). */
    fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix) {
        val r = DoubleArray(3) { visionMeasurementStdDevs[it, 0] * visionMeasurementStdDevs[it, 0] }

        // Closed-form steady-state Kalman gain for a continuous filter with A = 0, C = I.
        for (row in 0 until 3) {
            visionK[row, row] = if (q[row, 0] == 0.0) 0.0 else q[row, 0] / (q[row, 0] + sqrt(q[row, 0] * r[row]))
        }
    }

    fun resetPosition(gyroAngle: Rotation2d, wheelPositions: WheelPositions, pose: Pose2d) {
        odometry.resetPosition(gyroAngle, wheelPositions, pose)
        odometryPoseBuffer.clear()
        visionUpdates.clear()
        estimatedPosition = odometry.pose
    }

    fun resetPose(pose: Pose2d) {
        odometry.resetPose(pose)
        odometryPoseBuffer.clear()
        visionUpdates.clear()
        estimatedPosition = odometry.pose
    }

    fun resetTranslation(translation: Translation2d) {
        odometry.resetTranslation(translation)
        odometryPoseBuffer.clear()
        visionUpdates.clear()
        estimatedPosition = odometry.pose
    }

    fun resetRotation(rotation: Rotation2d) {
        odometry.resetRotation(rotation)
        odometryPoseBuffer.clear()
        visionUpdates.clear()
        estimatedPosition = odometry.pose
    }

    /** The estimated pose at [timestampSeconds], or null if there's no odometry history to sample from. */
    fun sampleAt(timestampSeconds: Double): Pose2d? {
        if (odometryPoseBuffer.internalBuffer.isEmpty()) return null

        val oldest = odometryPoseBuffer.internalBuffer.firstKey()
        val newest = odometryPoseBuffer.internalBuffer.lastKey()
        val clampedTime = timestampSeconds.coerceIn(oldest, newest)

        if (visionUpdates.isEmpty() || clampedTime < visionUpdates.firstKey()) {
            return odometryPoseBuffer.getSample(clampedTime)
        }

        val visionUpdate = visionUpdates[visionUpdates.floorKey(clampedTime)]!!
        val odometryEstimate = odometryPoseBuffer.getSample(clampedTime) ?: return null
        return visionUpdate.compensate(odometryEstimate)
    }

    /** Drops vision updates older than what any remaining odometry sample could still need. */
    private fun cleanUpVisionUpdates() {
        if (odometryPoseBuffer.internalBuffer.isEmpty()) return

        val oldestOdometryTimestamp = odometryPoseBuffer.internalBuffer.firstKey()
        if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) return

        val newestNeeded = visionUpdates.floorKey(oldestOdometryTimestamp)
        visionUpdates.headMap(newestNeeded, false).clear()
    }

    /**
     * Corrects the pose estimate toward [visionRobotPoseMeters], a global measurement (e.g. from
     * vision) taken at [timestampSeconds]. Can be called as infrequently as needed, as long as
     * [updateWithTime] is still called every loop. For stability, prefer only feeding in
     * measurements already within roughly a meter of the current estimate.
     */
    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        if (odometryPoseBuffer.internalBuffer.isEmpty() ||
            odometryPoseBuffer.internalBuffer.lastKey() - BUFFER_DURATION_SECONDS > timestampSeconds
        ) {
            return
        }

        cleanUpVisionUpdates()

        val odometrySample = odometryPoseBuffer.getSample(timestampSeconds) ?: return
        val visionSample = sampleAt(timestampSeconds) ?: return

        val twist = visionSample.log(visionRobotPoseMeters)
        val kTimesTwist = visionK * Matrix.vector(twist.dx, twist.dy, twist.dtheta)
        val scaledTwist = Twist2d(kTimesTwist[0, 0], kTimesTwist[1, 0], kTimesTwist[2, 0])

        val visionUpdate = VisionUpdate(visionSample.exp(scaledTwist), odometrySample)
        visionUpdates[timestampSeconds] = visionUpdate
        visionUpdates.tailMap(timestampSeconds, false).clear()

        estimatedPosition = visionUpdate.compensate(odometry.pose)
    }

    /** As the 2-arg overload, but also updating [setVisionMeasurementStdDevs] (which then applies to future calls too). */
    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs)
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds)
    }

    /** Integrates the latest [gyroAngle]/[wheelPositions] odometry reading, timestamped with the current wall-clock time. */
    fun update(gyroAngle: Rotation2d, wheelPositions: WheelPositions) =
        updateWithTime(System.nanoTime() / 1e9, gyroAngle, wheelPositions)

    /** As [update], but with an explicit [currentTimeSeconds] (e.g. matching your vision measurements' clock). */
    fun updateWithTime(currentTimeSeconds: Double, gyroAngle: Rotation2d, wheelPositions: WheelPositions): Pose2d {
        val odometryEstimate = odometry.update(gyroAngle, wheelPositions)
        odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate)

        estimatedPosition = if (visionUpdates.isEmpty()) {
            odometryEstimate
        } else {
            visionUpdates[visionUpdates.lastKey()]!!.compensate(odometryEstimate)
        }

        return estimatedPosition
    }

    private class VisionUpdate(val visionPose: Pose2d, val odometryPose: Pose2d) {
        /** [pose] re-expressed relative to [visionPose] instead of [odometryPose]. */
        fun compensate(pose: Pose2d): Pose2d {
            val delta = pose - odometryPose
            return visionPose + delta
        }
    }

    companion object {
        private const val BUFFER_DURATION_SECONDS = 1.5
    }
}
