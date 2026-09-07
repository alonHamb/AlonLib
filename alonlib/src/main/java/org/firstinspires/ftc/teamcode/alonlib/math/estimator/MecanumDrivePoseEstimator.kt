package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveOdometry
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveWheelPositions
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix

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
