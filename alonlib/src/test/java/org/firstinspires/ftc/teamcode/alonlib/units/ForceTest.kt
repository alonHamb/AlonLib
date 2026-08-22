package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class ForceTest {

    private val delta = 1e-3

    @Test
    fun `fromNewtons round-trips through every unit`() {
        val force = Force.fromNewtons(9.80665)
        assertEquals(9.80665, force.asNewtons, delta)
        assertEquals(1.0, force.asKilogramsForce, delta)
        assertEquals(2.20462262, force.asPoundsForce, delta)
        assertEquals(980665.0, force.asDynes, delta)
    }

    @Test
    fun `fromPoundsForce converts correctly`() {
        assertEquals(9.80665, Force.fromPoundsForce(2.20462262).asNewtons, delta)
    }

    @Test
    fun `fromKilogramsForce converts correctly`() {
        assertEquals(9.80665, Force.fromKilogramsForce(1.0).asNewtons, delta)
    }

    @Test
    fun `fromDynes converts correctly`() {
        assertEquals(1.0, Force.fromDynes(100_000.0).asNewtons, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Force.fromNewtons(Double.NaN).asNewtons, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Force.fromNewtons(Double.POSITIVE_INFINITY).asNewtons, delta)
        assertEquals(0.0, Force.fromNewtons(Double.NEGATIVE_INFINITY).asNewtons, delta)
    }

    @Test
    fun `toString reports newtons`() {
        assertEquals("Newtons(1.5)", Force.fromNewtons(1.5).toString())
    }

    @Test
    fun `compareTo orders by newtons`() {
        assertTrue(Force.fromNewtons(1.0) < Force.fromNewtons(2.0))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in newtons`() {
        assertEquals(3.0, (Force.fromNewtons(1.0) + Force.fromNewtons(2.0)).asNewtons, delta)
        assertEquals(1.0, (Force.fromNewtons(3.0) - Force.fromNewtons(2.0)).asNewtons, delta)
        assertEquals(2.0, (Force.fromNewtons(1.0) * 2.0).asNewtons, delta)
        assertEquals(0.5, (Force.fromNewtons(1.0) / 2.0).asNewtons, delta)
        assertEquals(-1.0, (-Force.fromNewtons(1.0)).asNewtons, delta)
    }
}
