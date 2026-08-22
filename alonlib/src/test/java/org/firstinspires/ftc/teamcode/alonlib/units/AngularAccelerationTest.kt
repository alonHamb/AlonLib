package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class AngularAccelerationTest {

    private val delta = 1e-4

    @Test
    fun `fromRadiansPerSecondSquared round-trips through every unit`() {
        val acceleration = AngularAcceleration.fromRadiansPerSecondSquared(Math.PI * 2.0)
        assertEquals(Math.PI * 2.0, acceleration.asRadiansPerSecondSquared, delta)
        assertEquals(360.0, acceleration.asDegreesPerSecondSquared, delta)
        assertEquals(1.0, acceleration.asRotationsPerSecondSquared, delta)
        assertEquals(60.0, acceleration.asRpmPerSecond, delta)
    }

    @Test
    fun `fromDegreesPerSecondSquared converts correctly`() {
        assertEquals(Math.PI, AngularAcceleration.fromDegreesPerSecondSquared(180.0).asRadiansPerSecondSquared, delta)
    }

    @Test
    fun `fromRotationsPerSecondSquared converts correctly`() {
        assertEquals(Math.PI * 2.0, AngularAcceleration.fromRotationsPerSecondSquared(1.0).asRadiansPerSecondSquared, delta)
    }

    @Test
    fun `fromRpmPerSecond converts correctly`() {
        assertEquals(Math.PI * 2.0, AngularAcceleration.fromRpmPerSecond(60.0).asRadiansPerSecondSquared, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, AngularAcceleration.fromRadiansPerSecondSquared(Double.NaN).asRadiansPerSecondSquared, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, AngularAcceleration.fromRadiansPerSecondSquared(Double.POSITIVE_INFINITY).asRadiansPerSecondSquared, delta)
        assertEquals(0.0, AngularAcceleration.fromRadiansPerSecondSquared(Double.NEGATIVE_INFINITY).asRadiansPerSecondSquared, delta)
    }

    @Test
    fun `toString reports radians per second squared`() {
        assertEquals("RadiansPerSecondSquared(1.5)", AngularAcceleration.fromRadiansPerSecondSquared(1.5).toString())
    }

    @Test
    fun `compareTo orders by radians per second squared`() {
        assertTrue(AngularAcceleration.fromRadiansPerSecondSquared(1.0) < AngularAcceleration.fromRadiansPerSecondSquared(2.0))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in radians per second squared`() {
        assertEquals(3.0, (AngularAcceleration.fromRadiansPerSecondSquared(1.0) + AngularAcceleration.fromRadiansPerSecondSquared(2.0)).asRadiansPerSecondSquared, delta)
        assertEquals(1.0, (AngularAcceleration.fromRadiansPerSecondSquared(3.0) - AngularAcceleration.fromRadiansPerSecondSquared(2.0)).asRadiansPerSecondSquared, delta)
        assertEquals(2.0, (AngularAcceleration.fromRadiansPerSecondSquared(1.0) * 2.0).asRadiansPerSecondSquared, delta)
        assertEquals(0.5, (AngularAcceleration.fromRadiansPerSecondSquared(1.0) / 2.0).asRadiansPerSecondSquared, delta)
        assertEquals(-1.0, (-AngularAcceleration.fromRadiansPerSecondSquared(1.0)).asRadiansPerSecondSquared, delta)
    }
}
