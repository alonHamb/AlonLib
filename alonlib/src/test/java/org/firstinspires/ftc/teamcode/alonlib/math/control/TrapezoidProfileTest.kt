package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TrapezoidProfileTest {

    private val delta = 1e-4

    @Test
    fun `reaches the goal position with zero velocity once totalTime has elapsed`() {
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity = 2.0, maxAcceleration = 1.0))
        val goal = TrapezoidProfile.State(10.0, 0.0)
        val start = TrapezoidProfile.State(0.0, 0.0)

        val totalTime = profile.calculate(0.0, start, goal).let { profile.totalTime() }
        val end = profile.calculate(totalTime, start, goal)

        assertEquals(10.0, end.position, delta)
        assertEquals(0.0, end.velocity, delta)
    }

    @Test
    fun `isFinished is false before totalTime and true at or after it`() {
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity = 2.0, maxAcceleration = 1.0))
        profile.calculate(0.0, TrapezoidProfile.State(0.0, 0.0), TrapezoidProfile.State(10.0, 0.0))
        val totalTime = profile.totalTime()

        assertTrue(!profile.isFinished(totalTime - 0.5))
        assertTrue(profile.isFinished(totalTime))
    }

    @Test
    fun `velocity never exceeds maxVelocity during a long move`() {
        val constraints = TrapezoidProfile.Constraints(maxVelocity = 2.0, maxAcceleration = 1.0)
        val profile = TrapezoidProfile(constraints)
        val start = TrapezoidProfile.State(0.0, 0.0)
        val goal = TrapezoidProfile.State(10.0, 0.0)

        var t = 0.0
        while (t < 10.0) {
            val state = profile.calculate(t, start, goal)
            assertTrue(state.velocity <= constraints.maxVelocity + delta)
            t += 0.1
        }
    }

    @Test
    fun `a goal within reach at constant velocity never leaves cruise speed`() {
        // Short move relative to the max velocity/acceleration -- this profile should never reach
        // maxVelocity at all (it's a "triangle" profile, not a full trapezoid).
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity = 100.0, maxAcceleration = 1.0))
        val goal = TrapezoidProfile.State(1.0, 0.0)
        val start = TrapezoidProfile.State(0.0, 0.0)

        profile.calculate(0.0, start, goal)
        val peakVelocity = (0..20).map { profile.calculate(it * 0.05, start, goal).velocity }.maxOrNull()!!
        assertTrue(peakVelocity < 100.0)
    }

    @Test
    fun `reversed goal (behind the start) still reaches the goal`() {
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity = 2.0, maxAcceleration = 1.0))
        val start = TrapezoidProfile.State(10.0, 0.0)
        val goal = TrapezoidProfile.State(0.0, 0.0)

        profile.calculate(0.0, start, goal)
        val end = profile.calculate(profile.totalTime(), start, goal)
        assertEquals(0.0, end.position, delta)
    }

    @Test
    fun `timeLeftUntil the goal position is approximately totalTime`() {
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity = 2.0, maxAcceleration = 1.0))
        val start = TrapezoidProfile.State(0.0, 0.0)
        val goal = TrapezoidProfile.State(10.0, 0.0)

        profile.calculate(0.0, start, goal)
        assertEquals(profile.totalTime(), profile.timeLeftUntil(10.0), 1e-2)
    }

    @Test
    fun `timeLeftUntil the start position is approximately zero`() {
        val profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity = 2.0, maxAcceleration = 1.0))
        val start = TrapezoidProfile.State(0.0, 0.0)
        val goal = TrapezoidProfile.State(10.0, 0.0)

        profile.calculate(0.0, start, goal)
        assertEquals(0.0, profile.timeLeftUntil(0.0), 1e-2)
    }
}
