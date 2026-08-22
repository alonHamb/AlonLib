package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class Rotation3dTest {

    private val delta = 1e-6

    @Test
    fun `roll pitch yaw round-trips through the x,y,z extraction`() {
        val roll = Math.toRadians(10.0)
        val pitch = Math.toRadians(20.0)
        val yaw = Math.toRadians(30.0)
        val rotation = Rotation3d(roll, pitch, yaw)

        assertEquals(roll, rotation.x, delta)
        assertEquals(pitch, rotation.y, delta)
        assertEquals(yaw, rotation.z, delta)
    }

    @Test
    fun `fromDegrees matches fromRadians`() {
        val a = Rotation3d.fromDegrees(10.0, 20.0, 30.0)
        val b = Rotation3d.fromRadians(Math.toRadians(10.0), Math.toRadians(20.0), Math.toRadians(30.0))
        assertEquals(a, b)
    }

    @Test
    fun `pure yaw projects onto Rotation2d unchanged`() {
        val rotation = Rotation3d(0.0, 0.0, Math.toRadians(45.0))
        assertEquals(45.0, rotation.toRotation2d().degrees, delta)
    }

    @Test
    fun `2D-to-3D bridge constructor is pure yaw`() {
        val rotation2d = Rotation2d.fromDegrees(60.0)
        val rotation3d = rotation2d.toRotation3d()
        assertEquals(60.0, rotation3d.toRotation2d().degrees, delta)
        assertEquals(0.0, rotation3d.x, delta)
        assertEquals(0.0, rotation3d.y, delta)
    }

    @Test
    fun `axis-angle constructor rotates a vector around that axis by that angle`() {
        // 90deg around the Z axis takes +X to +Y.
        val rotation = Rotation3d(Translation3d(0.0, 0.0, 1.0), Math.PI / 2.0)
        val rotated = Translation3d(1.0, 0.0, 0.0).rotateBy(rotation)
        assertEquals(0.0, rotated.x, delta)
        assertEquals(1.0, rotated.y, delta)
        assertEquals(0.0, rotated.z, delta)
    }

    @Test
    fun `getAxis and getAngle round-trip the axis-angle constructor`() {
        val axis = Translation3d(0.0, 0.0, 1.0)
        val angle = Math.PI / 3.0
        val rotation = Rotation3d(axis, angle)
        assertEquals(angle, rotation.angle, delta)
        assertEquals(axis.x, rotation.axis.x, delta)
        assertEquals(axis.y, rotation.axis.y, delta)
        assertEquals(axis.z, rotation.axis.z, delta)
    }

    @Test
    fun `rotateBy composes two rotations extrinsically`() {
        val a = Rotation3d.fromDegrees(90.0, 0.0, 0.0)
        val b = Rotation3d.fromDegrees(0.0, 0.0, 90.0)
        // Rotating a point by (a rotateBy b) should match rotating by a, then by b.
        val point = Translation3d(1.0, 0.0, 0.0)
        val composed = point.rotateBy(a).rotateBy(b)
        val direct = point.rotateBy(a.rotateBy(b))
        assertEquals(composed.x, direct.x, delta)
        assertEquals(composed.y, direct.y, delta)
        assertEquals(composed.z, direct.z, delta)
    }

    @Test
    fun `inverse undoes a rotation`() {
        val rotation = Rotation3d.fromDegrees(20.0, 30.0, 40.0)
        val point = Translation3d(1.0, 2.0, 3.0)
        val roundTripped = point.rotateBy(rotation).rotateBy(rotation.inverse())
        assertEquals(point.x, roundTripped.x, delta)
        assertEquals(point.y, roundTripped.y, delta)
        assertEquals(point.z, roundTripped.z, delta)
    }

    @Test
    fun `toMatrix and fromRotationMatrix round-trip a rotation`() {
        val rotation = Rotation3d.fromDegrees(15.0, -25.0, 65.0)
        val reconstructed = Rotation3d.fromRotationMatrix(rotation.toMatrix())
        assertEquals(rotation, reconstructed)
    }

    @Test
    fun `interpolate at the endpoints returns the endpoints themselves`() {
        val a = Rotation3d.kZero
        val b = Rotation3d.fromDegrees(0.0, 0.0, 90.0)
        assertEquals(a, a.interpolate(b, 0.0))
        assertEquals(b, a.interpolate(b, 1.0))
    }

    @Test
    fun `interpolate midway on a single-axis rotation is half the angle`() {
        val a = Rotation3d.kZero
        val b = Rotation3d.fromDegrees(0.0, 0.0, 90.0)
        assertEquals(45.0, a.interpolate(b, 0.5).toRotation2d().degrees, delta)
    }

    @Test
    fun `fromVectorToVector constructs the rotation that maps initial onto last`() {
        val initial = Translation3d(1.0, 0.0, 0.0)
        val last = Translation3d(0.0, 1.0, 0.0)
        val rotation = Rotation3d.fromVectorToVector(initial, last)
        val rotated = initial.rotateBy(rotation)
        assertEquals(last.x, rotated.x, delta)
        assertEquals(last.y, rotated.y, delta)
        assertEquals(last.z, rotated.z, delta)
    }

    @Test
    fun `kZero leaves vectors unchanged`() {
        val point = Translation3d(1.0, 2.0, 3.0)
        val rotated = point.rotateBy(Rotation3d.kZero)
        assertEquals(point.x, rotated.x, delta)
        assertEquals(point.y, rotated.y, delta)
        assertEquals(point.z, rotated.z, delta)
    }
}
