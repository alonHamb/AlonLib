package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/** Tracks a mecanum drivetrain's field pose from a gyro angle plus four wheel encoder distances. */
class MecanumDriveOdometry(
    private val kinematics: MecanumDriveKinematics,
    gyroAngle: Rotation2d,
    wheelPositions: MecanumDriveWheelPositions,
    initialPose: Pose2d = Pose2d.kZero,
) : Odometry<MecanumDriveWheelPositions>(kinematics, gyroAngle, wheelPositions, initialPose)
