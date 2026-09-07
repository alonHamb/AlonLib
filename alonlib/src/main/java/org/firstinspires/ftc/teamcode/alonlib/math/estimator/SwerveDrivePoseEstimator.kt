package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.SwerveDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.SwerveDriveOdometry
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.SwerveModulePosition
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix

/** [PoseEstimator] for a swerve drivetrain. */
class SwerveDrivePoseEstimator(
    kinematics: SwerveDriveKinematics,
    gyroAngle: Rotation2d,
    modulePositions: Array<SwerveModulePosition>,
    initialPose: Pose2d,
    stateStdDevs: Matrix,
    visionMeasurementStdDevs: Matrix,
) : PoseEstimator<Array<SwerveModulePosition>>(
        SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPose),
        stateStdDevs,
        visionMeasurementStdDevs,
    )
