package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Pose3dTest {

    private val delta = 1e-6

    @Test
    fun `transformBy applies a transform in this pose's rotated frame`() {
        val pose = Pose3d(0.0, 0.0, 0.0, Rotation3d.fromDegrees(0.0, 0.0, 90.0))
        val result = pose.transformBy(Transform3d(1.0, 0.0, 0.0, Rotation3d.kZero))
        assertEquals(0.0, result.x, delta)
        assertEquals(1.0, result.y, delta)
        assertEquals(0.0, result.z, delta)
    }

    @Test
    fun `minus produces the transform that plus would undo`() {
        val a = Pose3d(1.0, 2.0, 0.5, Rotation3d.fromDegrees(0.0, 0.0, 30.0))
        val b = Pose3d(4.0, 6.0, 1.5, Rotation3d.fromDegrees(0.0, 0.0, 90.0))
        val transform = b - a
        val reconstructed = a + transform
        assertEquals(b.x, reconstructed.x, delta)
        assertEquals(b.y, reconstructed.y, delta)
        assertEquals(b.z, reconstructed.z, delta)
        assertEquals(b.rotation, reconstructed.rotation)
    }

    @Test
    fun `relativeTo expresses this pose in another pose's frame`() {
        val origin = Pose3d(1.0, 1.0, 0.0, Rotation3d.fromDegrees(0.0, 0.0, 90.0))
        val absolute = Pose3d(1.0, 2.0, 0.0, Rotation3d.fromDegrees(0.0, 0.0, 90.0))
        val relative = absolute.relativeTo(origin)
        assertEquals(1.0, relative.x, delta)
        assertEquals(0.0, relative.y, delta)
        assertEquals(0.0, relative.z, delta)
        assertEquals(Rotation3d.kZero, relative.rotation)
    }

    @Test
    fun `toPose2d projects into the X-Y plane`() {
        val pose3d = Pose3d(1.0, 2.0, 5.0, Rotation3d.fromDegrees(10.0, 20.0, 45.0))
        val pose2d = pose3d.toPose2d()
        assertEquals(1.0, pose2d.x, delta)
        assertEquals(2.0, pose2d.y, delta)
        assertEquals(45.0, pose2d.rotation.degrees, delta)
    }

    @Test
    fun `2D-to-3D bridge constructor sets z to zero and rotation to pure yaw`() {
        val pose2d = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0))
        val pose3d = pose2d.toPose3d()
        assertEquals(0.0, pose3d.z, delta)
        assertEquals(45.0, pose3d.rotation.toRotation2d().degrees, delta)
    }

    @Test
    fun `interpolate at the endpoints returns the endpoints themselves`() {
        val a = Pose3d.kZero
        val b = Pose3d(10.0, 10.0, 10.0, Rotation3d.fromDegrees(0.0, 0.0, 90.0))
        assertEquals(a, a.interpolate(b, 0.0))
        assertEquals(b, a.interpolate(b, 1.0))
    }

    @Test
    fun `interpolate midway linearly interpolates translation`() {
        val a = Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero)
        val b = Pose3d(10.0, 0.0, 0.0, Rotation3d.kZero)
        val mid = a.interpolate(b, 0.5)
        assertEquals(5.0, mid.x, delta)
    }
}
