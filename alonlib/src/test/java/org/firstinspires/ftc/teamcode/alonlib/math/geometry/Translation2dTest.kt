package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Translation2dTest {

    private val delta = 1e-9

    @Test
    fun `default constructor is the origin`() {
        assertEquals(Translation2d.kZero, Translation2d())
    }

    @Test
    fun `polar constructor places the point at distance and angle from the origin`() {
        val t = Translation2d(5.0, Rotation2d.fromDegrees(90.0))
        assertEquals(0.0, t.x, delta)
        assertEquals(5.0, t.y, delta)
    }

    @Test
    fun `norm is the distance from the origin`() {
        assertEquals(5.0, Translation2d(3.0, 4.0).norm, delta)
    }

    @Test
    fun `angle is the direction from the origin`() {
        assertEquals(90.0, Translation2d(0.0, 5.0).angle.degrees, delta)
    }

    @Test
    fun `getDistance measures between two points`() {
        assertEquals(5.0, Translation2d(0.0, 0.0).getDistance(Translation2d(3.0, 4.0)), delta)
    }

    @Test
    fun `rotateBy rotates around the origin`() {
        val rotated = Translation2d(1.0, 0.0).rotateBy(Rotation2d.fromDegrees(90.0))
        assertEquals(0.0, rotated.x, delta)
        assertEquals(1.0, rotated.y, delta)
    }

    @Test
    fun `rotateAround rotates around an arbitrary point`() {
        val rotated = Translation2d(2.0, 0.0).rotateAround(Translation2d(1.0, 0.0), Rotation2d.fromDegrees(180.0))
        assertEquals(0.0, rotated.x, delta)
        assertEquals(0.0, rotated.y, delta)
    }

    @Test
    fun `plus, minus, unaryMinus, times, div operate component-wise`() {
        assertEquals(Translation2d(3.0, 5.0), Translation2d(1.0, 2.0) + Translation2d(2.0, 3.0))
        assertEquals(Translation2d(-1.0, -1.0), Translation2d(1.0, 2.0) - Translation2d(2.0, 3.0))
        assertEquals(Translation2d(-1.0, -2.0), -Translation2d(1.0, 2.0))
        assertEquals(Translation2d(2.0, 4.0), Translation2d(1.0, 2.0) * 2.0)
        assertEquals(Translation2d(0.5, 1.0), Translation2d(1.0, 2.0) / 2.0)
    }

    @Test
    fun `interpolate linearly interpolates each component`() {
        val a = Translation2d(0.0, 0.0)
        val b = Translation2d(10.0, 20.0)
        assertEquals(Translation2d(5.0, 10.0), a.interpolate(b, 0.5))
    }
}
