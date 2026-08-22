package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LinearVelocityTest {

    private val delta = 1e-4

    @Test
    fun `fromMetersPerSecond round-trips through every unit`() {
        val velocity = LinearVelocity.fromMetersPerSecond(1.0)
        assertEquals(1.0, velocity.asMetersPerSecond, delta)
        assertEquals(3.28084, velocity.asFeetPerSecond, delta)
        assertEquals(39.3700787402, velocity.asInchesPerSecond, delta)
        assertEquals(100.0, velocity.asCentimetersPerSecond, delta)
        assertEquals(1000.0, velocity.asMillimetersPerSecond, delta)
        assertEquals(3.6, velocity.asKilometersPerHour, delta)
        assertEquals(2.23693632, velocity.asMilesPerHour, delta)
    }

    @Test
    fun `fromFeetPerSecond converts correctly`() {
        assertEquals(1.0, LinearVelocity.fromFeetPerSecond(3.28084).asMetersPerSecond, delta)
    }

    @Test
    fun `fromKilometersPerHour converts correctly`() {
        assertEquals(1.0, LinearVelocity.fromKilometersPerHour(3.6).asMetersPerSecond, delta)
    }

    @Test
    fun `fromMilesPerHour converts correctly`() {
        assertEquals(1.0, LinearVelocity.fromMilesPerHour(2.23693632).asMetersPerSecond, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, LinearVelocity.fromMetersPerSecond(Double.NaN).asMetersPerSecond, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, LinearVelocity.fromMetersPerSecond(Double.POSITIVE_INFINITY).asMetersPerSecond, delta)
        assertEquals(0.0, LinearVelocity.fromMetersPerSecond(Double.NEGATIVE_INFINITY).asMetersPerSecond, delta)
    }

    @Test
    fun `toString reports meters per second`() {
        assertEquals("MetersPerSecond(1.5)", LinearVelocity.fromMetersPerSecond(1.5).toString())
    }

    @Test
    fun `compareTo orders by meters per second`() {
        assertTrue(LinearVelocity.fromMetersPerSecond(1.0) < LinearVelocity.fromMetersPerSecond(2.0))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in meters per second`() {
        assertEquals(1.5, (LinearVelocity.fromMetersPerSecond(1.0) + LinearVelocity.fromCentimetersPerSecond(50.0)).asMetersPerSecond, delta)
        assertEquals(0.5, (LinearVelocity.fromMetersPerSecond(1.0) - LinearVelocity.fromCentimetersPerSecond(50.0)).asMetersPerSecond, delta)
        assertEquals(2.0, (LinearVelocity.fromMetersPerSecond(1.0) * 2.0).asMetersPerSecond, delta)
        assertEquals(0.5, (LinearVelocity.fromMetersPerSecond(1.0) / 2.0).asMetersPerSecond, delta)
        assertEquals(-1.0, (-LinearVelocity.fromMetersPerSecond(1.0)).asMetersPerSecond, delta)
    }
}
