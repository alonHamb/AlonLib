package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PercentageTest {

    private val delta = 1e-6

    @Test
    fun `fromFraction round-trips through every unit`() {
        val percentage = Percentage.fromFraction(0.5)
        assertEquals(0.5, percentage.asFraction, delta)
        assertEquals(50.0, percentage.asPercent, delta)
        assertEquals(500.0, percentage.asPermille, delta)
        assertEquals(5000.0, percentage.asBasisPoints, delta)
    }

    @Test
    fun `fromPercent converts correctly`() {
        assertEquals(0.5, Percentage.fromPercent(50.0).asFraction, delta)
    }

    @Test
    fun `fromPermille converts correctly`() {
        assertEquals(0.5, Percentage.fromPermille(500.0).asFraction, delta)
    }

    @Test
    fun `fromBasisPoints converts correctly`() {
        assertEquals(0.5, Percentage.fromBasisPoints(5000.0).asFraction, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Percentage.fromFraction(Double.NaN).asFraction, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Percentage.fromFraction(Double.POSITIVE_INFINITY).asFraction, delta)
        assertEquals(0.0, Percentage.fromFraction(Double.NEGATIVE_INFINITY).asFraction, delta)
    }

    @Test
    fun `toString reports percent`() {
        assertEquals("Percentage(50.0%)", Percentage.fromFraction(0.5).toString())
    }

    @Test
    fun `compareTo orders by fraction`() {
        assertTrue(Percentage.fromFraction(0.1) < Percentage.fromFraction(0.2))
        assertEquals(0, Percentage.fromFraction(0.5).compareTo(Percentage.fromPercent(50.0)))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate on the fraction`() {
        assertEquals(0.75, (Percentage.fromFraction(0.5) + Percentage.fromPercent(25.0)).asFraction, delta)
        assertEquals(0.25, (Percentage.fromFraction(0.5) - Percentage.fromPercent(25.0)).asFraction, delta)
        assertEquals(1.0, (Percentage.fromFraction(0.5) * 2.0).asFraction, delta)
        assertEquals(0.25, (Percentage.fromFraction(0.5) / 2.0).asFraction, delta)
        assertEquals(-0.5, (-Percentage.fromFraction(0.5)).asFraction, delta)
    }

    @Test
    fun `coerceIn clamps to the given Percentage bounds`() {
        val min = Percentage.fromFraction(-1.0)
        val max = Percentage.fromFraction(1.0)
        assertEquals(1.0, Percentage.fromFraction(2.0).coerceIn(min, max).asFraction, delta)
        assertEquals(-1.0, Percentage.fromFraction(-2.0).coerceIn(min, max).asFraction, delta)
        assertEquals(0.5, Percentage.fromFraction(0.5).coerceIn(min, max).asFraction, delta)
    }
}
