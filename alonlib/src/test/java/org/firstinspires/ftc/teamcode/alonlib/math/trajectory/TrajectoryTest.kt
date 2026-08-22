package org.firstinspires.ftc.teamcode.alonlib.math.trajectory

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Transform2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.junit.Assert.assertEquals
import org.junit.Test

class TrajectoryTest {

    private val delta = 1e-6

    // Kinematically consistent (s = v0*t + 0.5*a*t^2) so that interpolate()'s dead-reckoning matches the stored poses.
    private val states = listOf(
        Trajectory.State(0.0, 0.0, 1.0, Pose2d(0.0, 0.0, Rotation2d.kZero), 0.0),
        Trajectory.State(1.0, 1.0, -1.0, Pose2d(0.5, 0.0, Rotation2d.kZero), 0.0),
        Trajectory.State(2.0, 0.0, 0.0, Pose2d(1.0, 0.0, Rotation2d.kZero), 0.0),
    )
    private val trajectory = Trajectory(states)

    @Test
    fun `totalTimeSeconds and initialPose come from the state list`() {
        assertEquals(2.0, trajectory.totalTimeSeconds, delta)
        assertEquals(0.0, trajectory.initialPose.x, delta)
    }

    @Test
    fun `sample before the first state clamps to it`() {
        assertEquals(states[0].pose.x, trajectory.sample(-1.0).pose.x, delta)
    }

    @Test
    fun `sample after the last state clamps to it`() {
        assertEquals(states.last().pose.x, trajectory.sample(5.0).pose.x, delta)
    }

    @Test
    fun `sample exactly on a state reconstructs its pose via dead reckoning`() {
        assertEquals(0.5, trajectory.sample(1.0).pose.x, delta)
    }

    @Test
    fun `sample midway interpolates position between neighboring states`() {
        val sampled = trajectory.sample(0.5)
        assertEquals(0.125, sampled.pose.x, delta)
    }

    @Test
    fun `transformBy shifts every pose by the transform relative to the first pose`() {
        val transformed = trajectory.transformBy(Transform2d(Translation2d(5.0, 0.0), Rotation2d.kZero))
        assertEquals(5.0, transformed.states[0].pose.x, delta)
        assertEquals(6.0, transformed.states.last().pose.x, delta)
    }

    @Test
    fun `relativeTo re-expresses every pose relative to the given pose`() {
        val relative = trajectory.relativeTo(Pose2d(1.0, 0.0, Rotation2d.kZero))
        assertEquals(-1.0, relative.states[0].pose.x, delta)
        assertEquals(0.0, relative.states.last().pose.x, delta)
    }

    @Test
    fun `concatenate appends the other trajectory with shifted timestamps and drops its first state`() {
        val other = Trajectory(
            listOf(
                Trajectory.State(0.0, 1.0, 0.0, Pose2d(2.0, 0.0, Rotation2d.kZero), 0.0),
                Trajectory.State(1.0, 1.0, 0.0, Pose2d(3.0, 0.0, Rotation2d.kZero), 0.0),
            ),
        )
        val combined = trajectory.concatenate(other)

        assertEquals(states.size + 1, combined.states.size)
        assertEquals(3.0, combined.totalTimeSeconds, delta)
        assertEquals(3.0, combined.states.last().pose.x, delta)
    }

    @Test
    fun `concatenate onto an empty trajectory returns the other trajectory`() {
        val empty = Trajectory()
        assertEquals(trajectory.states.size, empty.concatenate(trajectory).states.size)
    }
}
