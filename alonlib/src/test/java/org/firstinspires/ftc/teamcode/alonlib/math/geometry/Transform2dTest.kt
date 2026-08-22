package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Transform2dTest {

    private val delta = 1e-9

    @Test
    fun `constructor from initial and last poses is the relative transform between them`() {
        val initial = Pose2d(1.0, 1.0, Rotation2d.kZero)
        val last = Pose2d(3.0, 1.0, Rotation2d.kZero)
        val transform = Transform2d(initial, last)
        assertEquals(2.0, transform.x, delta)
        assertEquals(0.0, transform.y, delta)
    }

    @Test
    fun `inverse undoes the transform`() {
        val transform = Transform2d(2.0, 3.0, Rotation2d.fromDegrees(45.0))
        val pose = Pose2d.kZero.transformBy(transform).transformBy(transform.inverse())
        assertEquals(0.0, pose.x, delta)
        assertEquals(0.0, pose.y, delta)
        assertEquals(0.0, pose.rotation.degrees, delta)
    }

    @Test
    fun `plus composes two transforms end to end`() {
        val a = Transform2d(1.0, 0.0, Rotation2d.kZero)
        val b = Transform2d(0.0, 1.0, Rotation2d.kZero)
        val composed = a + b
        val applied = Pose2d.kZero.transformBy(composed)
        assertEquals(1.0, applied.x, delta)
        assertEquals(1.0, applied.y, delta)
    }

    @Test
    fun `times and div scale translation and rotation`() {
        val transform = Transform2d(2.0, 4.0, Rotation2d.fromDegrees(30.0))
        assertEquals(4.0, (transform * 2.0).x, delta)
        assertEquals(60.0, (transform * 2.0).rotation.degrees, delta)
        assertEquals(1.0, (transform / 2.0).x, delta)
    }
}
