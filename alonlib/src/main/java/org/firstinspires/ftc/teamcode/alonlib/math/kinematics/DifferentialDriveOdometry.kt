package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/**
 * Tracks a differential drivetrain's field pose from a gyro angle plus left/right encoder
 * distances. Reset both encoders to zero before constructing this (or any subsequent
 * [resetPosition] call needs them re-zeroed too).
 */
class DifferentialDriveOdometry(
    gyroAngle: Rotation2d,
    leftDistanceMeters: Double,
    rightDistanceMeters: Double,
    initialPose: Pose2d = Pose2d.kZero,
) : Odometry<DifferentialDriveWheelPositions>(
        DifferentialDriveKinematics(1.0),
        gyroAngle,
        DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters),
        initialPose,
    ) {

    fun resetPosition(gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double, pose: Pose2d) =
        resetPosition(gyroAngle, DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters), pose)

    fun update(gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double) =
        update(gyroAngle, DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters))
}
