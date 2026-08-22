package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/** Tracks a swerve drivetrain's field pose from a gyro angle plus each module's encoder distance + angle. */
class SwerveDriveOdometry(
    private val kinematics: SwerveDriveKinematics,
    gyroAngle: Rotation2d,
    modulePositions: Array<SwerveModulePosition>,
    initialPose: Pose2d = Pose2d.kZero,
) : Odometry<Array<SwerveModulePosition>>(kinematics, gyroAngle, modulePositions, initialPose)
