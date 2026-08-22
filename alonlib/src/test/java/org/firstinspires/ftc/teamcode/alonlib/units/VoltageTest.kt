package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class VoltageTest {

    private val delta = 1e-6

    @Test
    fun `fromVolts round-trips through every unit`() {
        val voltage = Voltage.fromVolts(1.0)
        assertEquals(1.0, voltage.asVolts, delta)
        assertEquals(1000.0, voltage.asMillivolts, delta)
        assertEquals(1_000_000.0, voltage.asMicrovolts, delta)
        assertEquals(0.001, voltage.asKilovolts, delta)
    }

    @Test
    fun `fromMillivolts converts correctly`() {
        assertEquals(1.0, Voltage.fromMillivolts(1000.0).asVolts, delta)
    }

    @Test
    fun `fromMicrovolts converts correctly`() {
        assertEquals(1.0, Voltage.fromMicrovolts(1_000_000.0).asVolts, delta)
    }

    @Test
    fun `fromKilovolts converts correctly`() {
        assertEquals(1000.0, Voltage.fromKilovolts(1.0).asVolts, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Voltage.fromVolts(Double.NaN).asVolts, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Voltage.fromVolts(Double.POSITIVE_INFINITY).asVolts, delta)
        assertEquals(0.0, Voltage.fromVolts(Double.NEGATIVE_INFINITY).asVolts, delta)
    }

    @Test
    fun `toString reports volts`() {
        assertEquals("Volts(12.0)", Voltage.fromVolts(12.0).toString())
    }

    @Test
    fun `compareTo orders by volts`() {
        assertTrue(Voltage.fromVolts(1.0) < Voltage.fromVolts(2.0))
        assertEquals(0, Voltage.fromVolts(1.0).compareTo(Voltage.fromMillivolts(1000.0)))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in volts`() {
        assertEquals(1.5, (Voltage.fromVolts(1.0) + Voltage.fromMillivolts(500.0)).asVolts, delta)
        assertEquals(0.5, (Voltage.fromVolts(1.0) - Voltage.fromMillivolts(500.0)).asVolts, delta)
        assertEquals(2.0, (Voltage.fromVolts(1.0) * 2.0).asVolts, delta)
        assertEquals(0.5, (Voltage.fromVolts(1.0) / 2.0).asVolts, delta)
        assertEquals(-1.0, (-Voltage.fromVolts(1.0)).asVolts, delta)
    }
}
