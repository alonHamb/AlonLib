package org.firstinspires.ftc.teamcode.alonlib.purepursuit

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.EndWaypoint
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.StartWaypoint
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class PathTest {

    @Test
    fun `init throws for a path missing a start or end waypoint`() {
        val path = Path(StartWaypoint(Pose2d(0.0, 0.0, Rotation2d.kZero)))
        assertFalse(path.isLegalPath())
    }

    @Test
    fun `a straight-line path drives from start to end when simulated as a simple integrator`() {
        val path = Path(
            StartWaypoint(Pose2d(0.0, 0.0, Rotation2d.kZero)),
            EndWaypoint(
                Pose2d(3.0, 0.0, Rotation2d.kZero),
                movementSpeed = 1.0,
                turnSpeed = 1.0,
                followRadius = 0.5,
                positionBuffer = 0.05,
                rotationBuffer = 0.1,
            ),
        )
        path.init()

        var x = 0.0
        var y = 0.0
        var heading = 0.0
        val dt = 0.02

        var iterations = 0
        while (!path.isFinished() && iterations < 5000) {
            val powers = path.loop(x, y, heading)
            // Treat the returned [strafe, forward, turn] powers as simple field-relative velocity commands.
            x += powers[0] * dt
            y += powers[1] * dt
            heading += powers[2] * dt
            iterations++
        }

        assertTrue("path should finish within the iteration budget", path.isFinished())
        assertFalse(path.timedOut)
        assertEquals(3.0, x, 0.1)
        assertEquals(0.0, y, 0.1)
    }

    @Test
    fun `reset clears timedOut and re-arms an already-finished path`() {
        val path = Path(
            StartWaypoint(Pose2d(0.0, 0.0, Rotation2d.kZero)),
            EndWaypoint(Pose2d(0.01, 0.0, Rotation2d.kZero), followRadius = 0.5, positionBuffer = 0.5, rotationBuffer = 3.2),
        )
        path.init()
        path.loop(0.0, 0.0, 0.0)
        assertTrue(path.isFinished())

        path.reset()
        assertFalse(path.isFinished())
    }
}
