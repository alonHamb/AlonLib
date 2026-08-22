package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class QuaternionTest {

    private val delta = 1e-9

    @Test
    fun `default constructor is the identity quaternion`() {
        val q = Quaternion()
        assertEquals(1.0, q.w, delta)
        assertEquals(0.0, q.x, delta)
        assertEquals(0.0, q.y, delta)
        assertEquals(0.0, q.z, delta)
    }

    @Test
    fun `times with the identity quaternion is a no-op`() {
        val q = Quaternion(0.5, 0.5, 0.5, 0.5)
        assertEquals(q, q * Quaternion())
        assertEquals(q, Quaternion() * q)
    }

    @Test
    fun `times composes two 90deg rotations about Z into a 180deg rotation`() {
        // (cos45, 0, 0, sin45) represents a 90deg rotation about +Z.
        val quarterTurnZ = Quaternion(kotlin.math.cos(Math.PI / 4.0), 0.0, 0.0, kotlin.math.sin(Math.PI / 4.0))
        val halfTurnZ = quarterTurnZ * quarterTurnZ
        assertEquals(0.0, halfTurnZ.x, delta)
        assertEquals(0.0, halfTurnZ.y, delta)
        assertEquals(1.0, kotlin.math.abs(halfTurnZ.z), delta)
    }

    @Test
    fun `conjugate negates the vector part`() {
        val q = Quaternion(1.0, 2.0, 3.0, 4.0)
        assertEquals(Quaternion(1.0, -2.0, -3.0, -4.0), q.conjugate())
    }

    @Test
    fun `inverse composed with the original is the identity`() {
        val q = Quaternion(1.0, 2.0, 3.0, 4.0)
        val result = q * q.inverse()
        assertEquals(1.0, result.w, delta)
        assertEquals(0.0, result.x, delta)
        assertEquals(0.0, result.y, delta)
        assertEquals(0.0, result.z, delta)
    }

    @Test
    fun `normalize produces a unit-norm quaternion pointing the same direction`() {
        val normalized = Quaternion(2.0, 0.0, 0.0, 0.0).normalize()
        assertEquals(1.0, normalized.norm(), delta)
        assertEquals(1.0, normalized.w, delta)
    }

    @Test
    fun `exp and log round-trip a quaternion`() {
        val q = Quaternion(0.1, 0.2, -0.3, 0.4)
        val roundTripped = q.log().exp()
        assertEquals(q.w, roundTripped.w, 1e-6)
        assertEquals(q.x, roundTripped.x, 1e-6)
        assertEquals(q.y, roundTripped.y, 1e-6)
        assertEquals(q.z, roundTripped.z, 1e-6)
    }

    @Test
    fun `dot of a quaternion with itself is its norm squared`() {
        val q = Quaternion(1.0, 2.0, 3.0, 4.0)
        assertEquals(q.norm() * q.norm(), q.dot(q), delta)
    }
}
