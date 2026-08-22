package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LinearAccelerationTest {

    private val delta = 1e-4

    @Test
    fun `fromMetersPerSecondSquared round-trips through every unit`() {
        val acceleration = LinearAcceleration.fromMetersPerSecondSquared(9.80665)
        assertEquals(9.80665, acceleration.asMetersPerSecondSquared, delta)
        assertEquals(1.0, acceleration.asGs, delta)
    }

    @Test
    fun `fromFeetPerSecondSquared converts correctly`() {
        assertEquals(1.0, LinearAcceleration.fromFeetPerSecondSquared(3.28084).asMetersPerSecondSquared, delta)
    }

    @Test
    fun `fromInchesPerSecondSquared converts correctly`() {
        assertEquals(1.0, LinearAcceleration.fromInchesPerSecondSquared(39.3700787402).asMetersPerSecondSquared, delta)
    }

    @Test
    fun `fromGs converts correctly`() {
        assertEquals(9.80665, LinearAcceleration.fromGs(1.0).asMetersPerSecondSquared, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, LinearAcceleration.fromMetersPerSecondSquared(Double.NaN).asMetersPerSecondSquared, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, LinearAcceleration.fromMetersPerSecondSquared(Double.POSITIVE_INFINITY).asMetersPerSecondSquared, delta)
        assertEquals(0.0, LinearAcceleration.fromMetersPerSecondSquared(Double.NEGATIVE_INFINITY).asMetersPerSecondSquared, delta)
    }

    @Test
    fun `toString reports meters per second squared`() {
        assertEquals("MetersPerSecondSquared(1.5)", LinearAcceleration.fromMetersPerSecondSquared(1.5).toString())
    }

    @Test
    fun `compareTo orders by meters per second squared`() {
        assertTrue(LinearAcceleration.fromMetersPerSecondSquared(1.0) < LinearAcceleration.fromMetersPerSecondSquared(2.0))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in meters per second squared`() {
        assertEquals(3.0, (LinearAcceleration.fromMetersPerSecondSquared(1.0) + LinearAcceleration.fromMetersPerSecondSquared(2.0)).asMetersPerSecondSquared, delta)
        assertEquals(1.0, (LinearAcceleration.fromMetersPerSecondSquared(3.0) - LinearAcceleration.fromMetersPerSecondSquared(2.0)).asMetersPerSecondSquared, delta)
        assertEquals(2.0, (LinearAcceleration.fromMetersPerSecondSquared(1.0) * 2.0).asMetersPerSecondSquared, delta)
        assertEquals(0.5, (LinearAcceleration.fromMetersPerSecondSquared(1.0) / 2.0).asMetersPerSecondSquared, delta)
        assertEquals(-1.0, (-LinearAcceleration.fromMetersPerSecondSquared(1.0)).asMetersPerSecondSquared, delta)
    }
}
