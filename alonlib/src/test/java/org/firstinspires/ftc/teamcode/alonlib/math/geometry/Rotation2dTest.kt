package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Rotation2dTest {

    private val delta = 1e-9

    @Test
    fun `fromDegrees and fromRadians and fromRotations agree`() {
        assertEquals(Math.PI / 2.0, Rotation2d.fromDegrees(90.0).radians, delta)
        assertEquals(90.0, Rotation2d.fromRadians(Math.PI / 2.0).degrees, delta)
        assertEquals(90.0, Rotation2d.fromRotations(0.25).degrees, delta)
    }

    @Test
    fun `vector constructor takes the angle of (x,y)`() {
        assertEquals(45.0, Rotation2d(1.0, 1.0).degrees, delta)
        assertEquals(0.0, Rotation2d(0.0, 0.0).degrees, delta)
    }

    @Test
    fun `cos and sin and tan match the stored angle`() {
        val r = Rotation2d.fromDegrees(60.0)
        assertEquals(kotlin.math.cos(r.radians), r.cos, delta)
        assertEquals(kotlin.math.sin(r.radians), r.sin, delta)
        assertEquals(kotlin.math.tan(r.radians), r.tan, delta)
    }

    @Test
    fun `plus composes rotations by adding angles`() {
        assertEquals(120.0, (Rotation2d.fromDegrees(90.0) + Rotation2d.fromDegrees(30.0)).degrees, delta)
    }

    @Test
    fun `minus is the inverse of plus`() {
        assertEquals(60.0, (Rotation2d.fromDegrees(90.0) - Rotation2d.fromDegrees(30.0)).degrees, delta)
    }

    @Test
    fun `unaryMinus negates the angle`() {
        assertEquals(-90.0, (-Rotation2d.fromDegrees(90.0)).degrees, delta)
    }

    @Test
    fun `times and div scale the angle`() {
        assertEquals(180.0, (Rotation2d.fromDegrees(90.0) * 2.0).degrees, delta)
        assertEquals(45.0, (Rotation2d.fromDegrees(90.0) / 2.0).degrees, delta)
    }

    @Test
    fun `rotateBy composes cos-sin like plus`() {
        val a = Rotation2d.fromDegrees(30.0)
        val b = Rotation2d.fromDegrees(60.0)
        assertEquals(90.0, a.rotateBy(b).degrees, delta)
    }

    @Test
    fun `interpolate at t=0,5 is the angle halfway between`() {
        val a = Rotation2d.fromDegrees(0.0)
        val b = Rotation2d.fromDegrees(90.0)
        assertEquals(45.0, a.interpolate(b, 0.5).degrees, delta)
    }

    @Test
    fun `equals treats angles that differ by a full turn as equal`() {
        assertEquals(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(360.0))
        assertEquals(Rotation2d.fromDegrees(10.0), Rotation2d.fromDegrees(370.0))
    }

    @Test
    fun `kZero is the identity rotation`() {
        assertEquals(0.0, Rotation2d.kZero.radians, delta)
    }
}
