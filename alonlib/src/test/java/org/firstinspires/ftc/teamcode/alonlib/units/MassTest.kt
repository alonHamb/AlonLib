package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class MassTest {

    private val delta = 1e-4

    @Test
    fun `fromKilograms round-trips through every unit`() {
        val mass = Mass.fromKilograms(1.0)
        assertEquals(1.0, mass.asKilograms, delta)
        assertEquals(1000.0, mass.asGrams, delta)
        assertEquals(2.20462262, mass.asPounds, delta)
        assertEquals(35.27396195, mass.asOunces, delta)
    }

    @Test
    fun `fromGrams converts correctly`() {
        assertEquals(1.0, Mass.fromGrams(1000.0).asKilograms, delta)
    }

    @Test
    fun `fromPounds converts correctly`() {
        assertEquals(1.0, Mass.fromPounds(2.20462262).asKilograms, delta)
    }

    @Test
    fun `fromOunces converts correctly`() {
        assertEquals(1.0, Mass.fromOunces(35.27396195).asKilograms, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Mass.fromKilograms(Double.NaN).asKilograms, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Mass.fromKilograms(Double.POSITIVE_INFINITY).asKilograms, delta)
        assertEquals(0.0, Mass.fromKilograms(Double.NEGATIVE_INFINITY).asKilograms, delta)
    }

    @Test
    fun `toString reports kilograms`() {
        assertEquals("Kilograms(1.5)", Mass.fromKilograms(1.5).toString())
    }

    @Test
    fun `compareTo orders by kilograms`() {
        assertTrue(Mass.fromKilograms(1.0) < Mass.fromKilograms(2.0))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in kilograms`() {
        assertEquals(1.5, (Mass.fromKilograms(1.0) + Mass.fromGrams(500.0)).asKilograms, delta)
        assertEquals(0.5, (Mass.fromKilograms(1.0) - Mass.fromGrams(500.0)).asKilograms, delta)
        assertEquals(2.0, (Mass.fromKilograms(1.0) * 2.0).asKilograms, delta)
        assertEquals(0.5, (Mass.fromKilograms(1.0) / 2.0).asKilograms, delta)
        assertEquals(-1.0, (-Mass.fromKilograms(1.0)).asKilograms, delta)
    }
}
