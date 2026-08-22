package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Pose2dTest {

    private val delta = 1e-6

    @Test
    fun `transformBy applies a transform in this pose's rotated frame`() {
        // Facing +90deg, "forward 1m" in the transform should land at (0, 1), not (1, 0).
        val pose = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))
        val result = pose.transformBy(Transform2d(1.0, 0.0, Rotation2d.kZero))
        assertEquals(0.0, result.x, delta)
        assertEquals(1.0, result.y, delta)
    }

    @Test
    fun `minus produces the transform that plus would undo`() {
        val a = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0))
        val b = Pose2d(4.0, 6.0, Rotation2d.fromDegrees(90.0))
        val transform = b - a
        assertEquals(b.x, (a + transform).x, delta)
        assertEquals(b.y, (a + transform).y, delta)
        assertEquals(b.rotation.degrees, (a + transform).rotation.degrees, delta)
    }

    @Test
    fun `relativeTo expresses this pose in another pose's frame`() {
        val origin = Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0))
        val absolute = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0))
        // absolute is 1m "ahead" of origin along origin's +90deg heading.
        val relative = absolute.relativeTo(origin)
        assertEquals(1.0, relative.x, delta)
        assertEquals(0.0, relative.y, delta)
        assertEquals(0.0, relative.rotation.degrees, delta)
    }

    @Test
    fun `exp of a straight-line twist moves forward along the current heading`() {
        val pose = Pose2d(0.0, 0.0, Rotation2d.kZero)
        val result = pose.exp(Twist2d(dx = 1.0, dy = 0.0, dtheta = 0.0))
        assertEquals(1.0, result.x, delta)
        assertEquals(0.0, result.y, delta)
    }

    @Test
    fun `log is the inverse of exp for a straight-line motion`() {
        val start = Pose2d(0.0, 0.0, Rotation2d.kZero)
        val end = Pose2d(2.0, 0.0, Rotation2d.kZero)
        val twist = start.log(end)
        assertEquals(2.0, twist.dx, delta)
        assertEquals(0.0, twist.dy, delta)
        assertEquals(0.0, twist.dtheta, delta)
    }

    @Test
    fun `exp and log round-trip an arbitrary curved motion`() {
        val start = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(20.0))
        val end = Pose2d(4.0, -1.0, Rotation2d.fromDegrees(160.0))

        val twist = start.log(end)
        val reconstructed = start.exp(twist)

        assertEquals(end.x, reconstructed.x, delta)
        assertEquals(end.y, reconstructed.y, delta)
        assertEquals(end.rotation.degrees, reconstructed.rotation.degrees, delta)
    }

    @Test
    fun `exp of a pure rotation turns in place`() {
        val pose = Pose2d(5.0, 5.0, Rotation2d.kZero)
        val result = pose.exp(Twist2d(dx = 0.0, dy = 0.0, dtheta = Math.PI / 2.0))
        assertEquals(5.0, result.x, delta)
        assertEquals(5.0, result.y, delta)
        assertEquals(90.0, result.rotation.degrees, delta)
    }

    @Test
    fun `interpolate at the endpoints returns the endpoints themselves`() {
        val a = Pose2d(0.0, 0.0, Rotation2d.kZero)
        val b = Pose2d(10.0, 10.0, Rotation2d.fromDegrees(90.0))
        assertEquals(a, a.interpolate(b, 0.0))
        assertEquals(b, a.interpolate(b, 1.0))
    }

    @Test
    fun `interpolate midway is between the two poses`() {
        val a = Pose2d(0.0, 0.0, Rotation2d.kZero)
        val b = Pose2d(10.0, 0.0, Rotation2d.kZero)
        val mid = a.interpolate(b, 0.5)
        assertEquals(5.0, mid.x, delta)
        assertEquals(0.0, mid.y, delta)
    }
}
