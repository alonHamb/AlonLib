package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class CurrentTest {

    private val delta = 1e-6

    @Test
    fun `fromAmps round-trips through every unit`() {
        val current = Current.fromAmps(1.0)
        assertEquals(1.0, current.asAmps, delta)
        assertEquals(1000.0, current.asMilliamps, delta)
        assertEquals(1_000_000.0, current.asMicroamps, delta)
        assertEquals(0.001, current.asKiloamps, delta)
    }

    @Test
    fun `fromMilliamps converts correctly`() {
        assertEquals(1.0, Current.fromMilliamps(1000.0).asAmps, delta)
    }

    @Test
    fun `fromMicroamps converts correctly`() {
        assertEquals(1.0, Current.fromMicroamps(1_000_000.0).asAmps, delta)
    }

    @Test
    fun `fromKiloamps converts correctly`() {
        assertEquals(1000.0, Current.fromKiloamps(1.0).asAmps, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Current.fromAmps(Double.NaN).asAmps, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Current.fromAmps(Double.POSITIVE_INFINITY).asAmps, delta)
        assertEquals(0.0, Current.fromAmps(Double.NEGATIVE_INFINITY).asAmps, delta)
    }

    @Test
    fun `toString reports amps`() {
        assertEquals("Amps(5.0)", Current.fromAmps(5.0).toString())
    }

    @Test
    fun `compareTo orders by amps`() {
        assertTrue(Current.fromAmps(1.0) < Current.fromAmps(2.0))
        assertEquals(0, Current.fromAmps(1.0).compareTo(Current.fromMilliamps(1000.0)))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in amps`() {
        assertEquals(1.5, (Current.fromAmps(1.0) + Current.fromMilliamps(500.0)).asAmps, delta)
        assertEquals(0.5, (Current.fromAmps(1.0) - Current.fromMilliamps(500.0)).asAmps, delta)
        assertEquals(2.0, (Current.fromAmps(1.0) * 2.0).asAmps, delta)
        assertEquals(0.5, (Current.fromAmps(1.0) / 2.0).asAmps, delta)
        assertEquals(-1.0, (-Current.fromAmps(1.0)).asAmps, delta)
    }
}
