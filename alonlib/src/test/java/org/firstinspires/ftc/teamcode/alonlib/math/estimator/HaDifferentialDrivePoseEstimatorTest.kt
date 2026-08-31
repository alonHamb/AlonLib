package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class HaDifferentialDrivePoseEstimatorTest {

	private fun estimator() = DifferentialDrivePoseEstimator(
		DifferentialDriveKinematics(0.5),
		Rotation2d.kZero,
		0.0,
		0.0,
		Pose2d.kZero,
		Matrix.vector(0.02, 0.02, 0.01),
		Matrix.vector(0.1, 0.1, 0.1),
	)

	@Test
	fun `with no vision measurements, behaves exactly like plain odometry`() {
		val estimator = estimator()
		val pose = estimator.updateWithTime(0.0, Rotation2d.kZero, 1.0, 1.0)
		assertEquals(1.0, pose.x, 1e-9)
		assertEquals(1.0, estimator.estimatedPosition.x, 1e-9)
	}

	@Test
	fun `a vision measurement pulls the estimate toward it`() {
		val estimator = estimator()
		estimator.updateWithTime(0.0, Rotation2d.kZero, 0.0, 0.0)

		estimator.addVisionMeasurement(Pose2d(5.0, 0.0, Rotation2d.kZero), 0.0)

		assertTrue(estimator.estimatedPosition.x > 0.0)
		assertTrue(estimator.estimatedPosition.x < 5.0)
	}

	@Test
	fun `resetPose clears vision history and snaps to the given pose`() {
		val estimator = estimator()
		estimator.updateWithTime(0.0, Rotation2d.kZero, 1.0, 1.0)
		estimator.addVisionMeasurement(Pose2d(5.0, 0.0, Rotation2d.kZero), 0.0)

		estimator.resetPose(Pose2d(2.0, 3.0, Rotation2d.kZero))

		assertEquals(2.0, estimator.estimatedPosition.x, 1e-9)
		assertEquals(3.0, estimator.estimatedPosition.y, 1e-9)
	}
}
