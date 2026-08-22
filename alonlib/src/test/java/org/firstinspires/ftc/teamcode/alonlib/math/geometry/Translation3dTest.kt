package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Translation3dTest {

    private val delta = 1e-9

    @Test
    fun `default constructor is the origin`() {
        assertEquals(Translation3d.kZero, Translation3d())
    }

    @Test
    fun `2D-to-3D bridge constructor sets z to zero`() {
        val t = Translation2d(3.0, 4.0).toTranslation3d()
        assertEquals(3.0, t.x, delta)
        assertEquals(4.0, t.y, delta)
        assertEquals(0.0, t.z, delta)
    }

    @Test
    fun `toTranslation2d projects into the X-Y plane`() {
        val t2d = Translation3d(1.0, 2.0, 3.0).toTranslation2d()
        assertEquals(1.0, t2d.x, delta)
        assertEquals(2.0, t2d.y, delta)
    }

    @Test
    fun `norm is the distance from the origin`() {
        assertEquals(3.0, Translation3d(1.0, 2.0, 2.0).norm, delta)
    }

    @Test
    fun `getDistance measures between two points`() {
        assertEquals(3.0, Translation3d(0.0, 0.0, 0.0).getDistance(Translation3d(1.0, 2.0, 2.0)), delta)
    }

    @Test
    fun `dot computes the dot product`() {
        assertEquals(32.0, Translation3d(1.0, 2.0, 3.0).dot(Translation3d(4.0, 5.0, 6.0)), delta)
    }

    @Test
    fun `cross computes a vector perpendicular to both inputs`() {
        val cross = Translation3d(1.0, 0.0, 0.0).cross(Translation3d(0.0, 1.0, 0.0))
        assertEquals(Translation3d(0.0, 0.0, 1.0), cross)
    }

    @Test
    fun `plus, minus, unaryMinus, times, div operate component-wise`() {
        assertEquals(Translation3d(4.0, 6.0, 8.0), Translation3d(1.0, 2.0, 3.0) + Translation3d(3.0, 4.0, 5.0))
        assertEquals(Translation3d(-2.0, -2.0, -2.0), Translation3d(1.0, 2.0, 3.0) - Translation3d(3.0, 4.0, 5.0))
        assertEquals(Translation3d(-1.0, -2.0, -3.0), -Translation3d(1.0, 2.0, 3.0))
        assertEquals(Translation3d(2.0, 4.0, 6.0), Translation3d(1.0, 2.0, 3.0) * 2.0)
        assertEquals(Translation3d(0.5, 1.0, 1.5), Translation3d(1.0, 2.0, 3.0) / 2.0)
    }

    @Test
    fun `interpolate linearly interpolates each component`() {
        val a = Translation3d(0.0, 0.0, 0.0)
        val b = Translation3d(10.0, 20.0, 30.0)
        assertEquals(Translation3d(5.0, 10.0, 15.0), a.interpolate(b, 0.5))
    }

    @Test
    fun `nearest returns the closest translation from a collection`() {
        val origin = Translation3d(0.0, 0.0, 0.0)
        val candidates = listOf(Translation3d(10.0, 0.0, 0.0), Translation3d(1.0, 0.0, 0.0), Translation3d(5.0, 0.0, 0.0))
        assertEquals(Translation3d(1.0, 0.0, 0.0), origin.nearest(candidates))
    }

    @Test
    fun `polar constructor at distance and angle matches rotateBy of the x-axis`() {
        val angle = Rotation3d(Translation3d(0.0, 0.0, 1.0), Math.PI / 2.0)
        val polar = Translation3d(5.0, angle)
        val rotated = Translation3d(5.0, 0.0, 0.0).rotateBy(angle)
        assertEquals(rotated.x, polar.x, 1e-9)
        assertEquals(rotated.y, polar.y, 1e-9)
        assertEquals(rotated.z, polar.z, 1e-9)
    }
}
