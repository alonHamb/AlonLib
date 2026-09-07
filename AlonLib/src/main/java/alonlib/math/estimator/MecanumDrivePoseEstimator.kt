package alonlib.math.estimator

import alonlib.math.geometry.Pose2d
import alonlib.math.geometry.Rotation2d
import alonlib.math.kinematics.MecanumDriveKinematics
import alonlib.math.kinematics.MecanumDriveOdometry
import alonlib.math.kinematics.MecanumDriveWheelPositions
import alonlib.math.system.Matrix

/** [PoseEstimator] for a mecanum drivetrain. */
class MecanumDrivePoseEstimator(
    kinematics: MecanumDriveKinematics,
    gyroAngle: Rotation2d,
    wheelPositions: MecanumDriveWheelPositions,
    initialPose: Pose2d,
    stateStdDevs: Matrix,
    visionMeasurementStdDevs: Matrix,
) : PoseEstimator<MecanumDriveWheelPositions>(
        MecanumDriveOdometry(kinematics, gyroAngle, wheelPositions, initialPose),
        stateStdDevs,
        visionMeasurementStdDevs,
    )
