package alonlib.math.kinematics

import alonlib.math.geometry.AngularPosition2d
import alonlib.math.geometry.Pose2d

/** Tracks a mecanum drivetrain's field pose from a gyro angle plus four wheel encoder distances. */
class MecanumDriveOdometry(
	kinematics: MecanumDriveKinematics,
	gyroAngle: AngularPosition2d,
	wheelPositions: MecanumDriveWheelPositions,
	initialPose: Pose2d = Pose2d.kZero,
) : Odometry<MecanumDriveWheelPositions>(kinematics, gyroAngle, wheelPositions, initialPose)
