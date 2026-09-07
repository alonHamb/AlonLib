package alonlib.math.estimator

import alonlib.math.geometry.Pose2d
import alonlib.math.geometry.Rotation2d
import alonlib.math.kinematics.DifferentialDriveKinematics
import alonlib.math.kinematics.DifferentialDriveOdometry
import alonlib.math.kinematics.DifferentialDriveWheelPositions
import alonlib.math.system.Matrix

/**
 * [PoseEstimator] for a differential drivetrain. [kinematics] isn't actually used here (matching
 * upstream WPILib, which keeps it only for API-shape parity with the other drivetrains' pose
 * estimators) -- [DifferentialDriveOdometry] tracks translation from raw wheel-distance averages
 * and takes rotation from the gyro, neither of which depends on the trackwidth.
 */
class DifferentialDrivePoseEstimator(
    kinematics: DifferentialDriveKinematics,
    gyroAngle: Rotation2d,
    leftDistanceMeters: Double,
    rightDistanceMeters: Double,
    initialPose: Pose2d,
    stateStdDevs: Matrix,
    visionMeasurementStdDevs: Matrix,
) : PoseEstimator<DifferentialDriveWheelPositions>(
        DifferentialDriveOdometry(gyroAngle, leftDistanceMeters, rightDistanceMeters, initialPose),
        stateStdDevs,
        visionMeasurementStdDevs,
    ) {

    fun update(gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double) =
        update(gyroAngle, DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters))

    fun updateWithTime(currentTimeSeconds: Double, gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double) =
        updateWithTime(currentTimeSeconds, gyroAngle, DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters))
}
