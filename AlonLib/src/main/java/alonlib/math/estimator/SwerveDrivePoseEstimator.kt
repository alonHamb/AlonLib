package alonlib.math.estimator

import alonlib.math.geometry.Pose2d
import alonlib.math.geometry.AngularPositon
import alonlib.math.kinematics.SwerveDriveKinematics
import alonlib.math.kinematics.SwerveDriveOdometry
import alonlib.math.kinematics.SwerveModulePosition
import alonlib.math.system.Matrix

/** [PoseEstimator] for a swerve drivetrain. */
class SwerveDrivePoseEstimator(
	kinematics: SwerveDriveKinematics,
	gyroAngle: AngularPositon,
	modulePositions: Array<SwerveModulePosition>,
	initialPose: Pose2d,
	stateStdDevs: Matrix,
	visionMeasurementStdDevs: Matrix,
) : PoseEstimator<Array<SwerveModulePosition>>(
	SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPose),
	stateStdDevs,
	visionMeasurementStdDevs,
)
