package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Vector2dTest {

    private val delta = 1e-9

    @Test
    fun `default constructor is the zero vector`() {
        assertEquals(Vector2d(0.0, 0.0), Vector2d())
    }

    @Test
    fun `Pose2d constructor drops the rotation`() {
        val v = Vector2d(Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45.0)))
        assertEquals(3.0, v.x, delta)
        assertEquals(4.0, v.y, delta)
    }

    @Test
    fun `rotateBy rotates counter-clockwise by radians`() {
        val rotated = Vector2d(1.0, 0.0).rotateBy(Math.PI / 2.0)
        assertEquals(0.0, rotated.x, delta)
        assertEquals(1.0, rotated.y, delta)
    }

    @Test
    fun `angle returns the direction in radians`() {
        assertEquals(Math.PI / 2.0, Vector2d(0.0, 1.0).angle(), delta)
    }

    @Test
    fun `plus, minus, unaryMinus, times, div operate component-wise`() {
        assertEquals(Vector2d(3.0, 5.0), Vector2d(1.0, 2.0) + Vector2d(2.0, 3.0))
        assertEquals(Vector2d(-1.0, -1.0), Vector2d(1.0, 2.0) - Vector2d(2.0, 3.0))
        assertEquals(Vector2d(-1.0, -2.0), -Vector2d(1.0, 2.0))
        assertEquals(Vector2d(2.0, 4.0), Vector2d(1.0, 2.0) * 2.0)
        assertEquals(Vector2d(0.5, 1.0), Vector2d(1.0, 2.0) / 2.0)
    }

    @Test
    fun `dot computes the dot product`() {
        assertEquals(11.0, Vector2d(1.0, 2.0).dot(Vector2d(3.0, 4.0)), delta)
    }

    @Test
    fun `magnitude is the vector's length`() {
        assertEquals(5.0, Vector2d(3.0, 4.0).magnitude(), delta)
    }

    @Test
    fun `normalize scales to unit length while keeping direction`() {
        val normalized = Vector2d(3.0, 4.0).normalize()
        assertEquals(1.0, normalized.magnitude(), delta)
        assertEquals(0.6, normalized.x, delta)
        assertEquals(0.8, normalized.y, delta)
    }

    @Test
    fun `project projects this vector onto another`() {
        // Projecting (2, 2) onto the x-axis keeps only the x component.
        val projected = Vector2d(2.0, 2.0).project(Vector2d(1.0, 0.0))
        assertEquals(2.0, projected.x, delta)
        assertEquals(0.0, projected.y, delta)
    }

    @Test
    fun `scalarProject returns the scalar length of the projection`() {
        assertEquals(2.0, Vector2d(2.0, 2.0).scalarProject(Vector2d(1.0, 0.0)), delta)
    }
}
