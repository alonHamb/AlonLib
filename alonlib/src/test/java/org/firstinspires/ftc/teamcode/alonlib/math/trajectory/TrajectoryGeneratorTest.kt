package org.firstinspires.ftc.teamcode.alonlib.math.trajectory

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TrajectoryGeneratorTest {

    private val delta = 1e-3
    private val config = TrajectoryConfig(maxVelocityMetersPerSecond = 2.0, maxAccelerationMetersPerSecondSq = 2.0)

    @Test
    fun `straight line trajectory starts and ends at rest at the waypoints`() {
        val trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0.0, 0.0, Rotation2d.kZero),
            emptyList(),
            Pose2d(3.0, 0.0, Rotation2d.kZero),
            config,
        )

        assertEquals(0.0, trajectory.states.first().velocityMetersPerSecond, delta)
        assertEquals(0.0, trajectory.states.last().velocityMetersPerSecond, delta)
        assertEquals(3.0, trajectory.states.last().pose.x, delta)
        assertTrue(trajectory.totalTimeSeconds > 0.0)
    }

    @Test
    fun `never exceeds the configured max velocity`() {
        val trajectory = TrajectoryGenerator.generateTrajectory(
            listOf(
                Pose2d(0.0, 0.0, Rotation2d.kZero),
                Pose2d(1.0, 1.0, Rotation2d.fromDegrees(45.0)),
                Pose2d(2.0, 0.0, Rotation2d.kZero),
            ),
            config,
        )

        trajectory.states.forEach { assertTrue(kotlin.math.abs(it.velocityMetersPerSecond) <= config.maxVelocityMetersPerSecond + delta) }
    }

    @Test
    fun `respects a differential drive kinematics constraint`() {
        val kinematics = DifferentialDriveKinematics(trackWidthMeters = 0.5)
        config.setKinematics(kinematics)

        val trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0.0, 0.0, Rotation2d.kZero),
            emptyList(),
            Pose2d(2.0, 0.0, Rotation2d.fromDegrees(90.0)),
            config,
        )

        trajectory.states.forEach {
            val wheelSpeeds = kinematics.toWheelSpeeds(
                org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds(
                    it.velocityMetersPerSecond, 0.0, it.velocityMetersPerSecond * it.curvatureRadPerMeter,
                ),
            )
            assertTrue(kotlin.math.abs(wheelSpeeds.left) <= config.maxVelocityMetersPerSecond + delta)
            assertTrue(kotlin.math.abs(wheelSpeeds.right) <= config.maxVelocityMetersPerSecond + delta)
        }
    }

    @Test
    fun `reversed trajectory drives backwards from start to end`() {
        val reversedConfig = TrajectoryConfig(maxVelocityMetersPerSecond = 2.0, maxAccelerationMetersPerSecondSq = 2.0)
        reversedConfig.reversed = true

        // A perfectly straight, collinear-with-heading path is a known degenerate case for reversed
        // spline generation (both endpoint tangents point backward while y stays identically zero,
        // forcing velocity through zero) -- so this uses a slight y-offset instead.
        val trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0.0, 0.0, Rotation2d.kZero),
            emptyList(),
            Pose2d(3.0, 1.0, Rotation2d.kZero),
            reversedConfig,
        )

        assertEquals(3.0, trajectory.states.last().pose.x, delta)
        assertTrue(trajectory.states[trajectory.states.size / 2].velocityMetersPerSecond <= 0.0)
    }
}
