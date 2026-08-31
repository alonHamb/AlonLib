package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class HaDifferentialDriveVoltageConstraintTest {

	private val delta = 1e-6
	private val feedforward = SimpleMotorFeedforward(ks = 1.0, kv = 1.0, ka = 0.5)
	private val kinematics = DifferentialDriveKinematics(trackWidthMeters = 1.0)
	private val constraint = DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage = 10.0)

	@Test
	fun `imposes no velocity limit, only an acceleration one`() {
		assertEquals(Double.POSITIVE_INFINITY, constraint.getMaxVelocityMetersPerSecond(Pose2d.kZero, 0.0, 1.0), delta)
	}

	@Test
	fun `straight line acceleration bound matches the feedforward's achievable acceleration`() {
		val minMax = constraint.getMinMaxAccelerationMetersPerSecondSq(Pose2d.kZero, 0.0, 1.0)
		val expectedMax = feedforward.maxAchievableAcceleration(10.0, 1.0)
		val expectedMin = feedforward.minAchievableAcceleration(10.0, 1.0)

		assertEquals(expectedMax, minMax.maxAccelerationMetersPerSecondSq, delta)
		assertEquals(expectedMin, minMax.minAccelerationMetersPerSecondSq, delta)
	}

	@Test
	fun `turning tightens the acceleration bound relative to a straight line`() {
		val straight = constraint.getMinMaxAccelerationMetersPerSecondSq(Pose2d.kZero, 0.0, 1.0)
		val turning = constraint.getMinMaxAccelerationMetersPerSecondSq(Pose2d.kZero, 1.0, 1.0)

		assertTrue(turning.maxAccelerationMetersPerSecondSq < straight.maxAccelerationMetersPerSecondSq)
	}
}
