package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Transform3dTest {

    private val delta = 1e-9

    @Test
    fun `constructor from initial and last poses is the relative transform between them`() {
        val initial = Pose3d(1.0, 1.0, 0.0, Rotation3d.kZero)
        val last = Pose3d(3.0, 1.0, 0.0, Rotation3d.kZero)
        val transform = Transform3d(initial, last)
        assertEquals(2.0, transform.x, delta)
        assertEquals(0.0, transform.y, delta)
        assertEquals(0.0, transform.z, delta)
    }

    @Test
    fun `inverse undoes the transform`() {
        val transform = Transform3d(2.0, 3.0, 1.0, Rotation3d.fromDegrees(0.0, 0.0, 45.0))
        val pose = Pose3d.kZero.transformBy(transform).transformBy(transform.inverse())
        assertEquals(0.0, pose.x, delta)
        assertEquals(0.0, pose.y, delta)
        assertEquals(0.0, pose.z, delta)
        assertEquals(Rotation3d.kZero, pose.rotation)
    }

    @Test
    fun `plus composes two transforms end to end`() {
        val a = Transform3d(1.0, 0.0, 0.0, Rotation3d.kZero)
        val b = Transform3d(0.0, 1.0, 0.0, Rotation3d.kZero)
        val applied = Pose3d.kZero.transformBy(a + b)
        assertEquals(1.0, applied.x, delta)
        assertEquals(1.0, applied.y, delta)
    }

    @Test
    fun `2D-to-3D bridge constructor sets z to zero and rotation to pure yaw`() {
        val transform2d = Transform2d(1.0, 2.0, Rotation2d.fromDegrees(30.0))
        val transform3d = transform2d.toTransform3d()
        assertEquals(1.0, transform3d.x, delta)
        assertEquals(2.0, transform3d.y, delta)
        assertEquals(0.0, transform3d.z, delta)
        assertEquals(30.0, transform3d.rotation.toRotation2d().degrees, 1e-6)
    }
}
