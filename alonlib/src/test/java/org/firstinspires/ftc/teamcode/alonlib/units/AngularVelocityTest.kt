package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class AngularVelocityTest {

    private val delta = 1e-6

    @Test
    fun `fromRpm round-trips through every unit`() {
        val velocity = AngularVelocity.fromRpm(60.0)
        assertEquals(60.0, velocity.asRpm, delta)
        assertEquals(1.0, velocity.asRps, delta)
        assertEquals(Math.PI * 2.0, velocity.asRadPs, delta)
        assertEquals(360.0, velocity.asDegPs, delta)
    }

    @Test
    fun `fromRps converts correctly`() {
        assertEquals(60.0, AngularVelocity.fromRps(1.0).asRpm, delta)
    }

    @Test
    fun `fromRadPs converts correctly`() {
        assertEquals(60.0, AngularVelocity.fromRadPs(Math.PI * 2.0).asRpm, delta)
    }

    @Test
    fun `fromDegPs converts correctly`() {
        assertEquals(60.0, AngularVelocity.fromDegPs(360.0).asRpm, delta)
    }

    @Test
    fun `asMps converts using the wheel's circumference`() {
        // 60 rpm = 1 rev/s; a 1m-radius wheel has a 2*pi m circumference.
        val velocity = AngularVelocity.fromRpm(60.0)
        assertEquals(Math.PI * 2.0, velocity.asMps(Length.fromMeters(1.0)), delta)
    }

    @Test
    fun `fromMps is the inverse of asMps`() {
        val wheelRadius = Length.fromMeters(0.5)
        val velocity = AngularVelocity.fromMps(1.0, wheelRadius)
        assertEquals(1.0, velocity.asMps(wheelRadius), delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, AngularVelocity.fromRpm(Double.NaN).asRpm, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, AngularVelocity.fromRpm(Double.POSITIVE_INFINITY).asRpm, delta)
        assertEquals(0.0, AngularVelocity.fromRpm(Double.NEGATIVE_INFINITY).asRpm, delta)
    }

    @Test
    fun `absoluteValue drops the sign`() {
        assertEquals(60.0, AngularVelocity.fromRpm(-60.0).absoluteValue.asRpm, delta)
        assertEquals(60.0, AngularVelocity.fromRpm(60.0).absoluteValue.asRpm, delta)
    }

    @Test
    fun `toString reports rpm`() {
        assertEquals("RPM(60.0)", AngularVelocity.fromRpm(60.0).toString())
    }

    @Test
    fun `compareTo orders by rpm`() {
        assertTrue(AngularVelocity.fromRpm(1.0) < AngularVelocity.fromRpm(2.0))
        assertTrue(AngularVelocity.fromRpm(2.0) > AngularVelocity.fromRpm(1.0))
        assertEquals(0, AngularVelocity.fromRpm(60.0).compareTo(AngularVelocity.fromRps(1.0)))
    }

    @Test
    fun `plus and minus operate in rpm`() {
        val sum = AngularVelocity.fromRpm(60.0) + AngularVelocity.fromRps(1.0)
        assertEquals(120.0, sum.asRpm, delta)

        val difference = AngularVelocity.fromRpm(120.0) - AngularVelocity.fromRps(1.0)
        assertEquals(60.0, difference.asRpm, delta)
    }

    @Test
    fun `times and div scale the rpm value`() {
        assertEquals(120.0, (AngularVelocity.fromRpm(60.0) * 2.0).asRpm, delta)
        assertEquals(30.0, (AngularVelocity.fromRpm(60.0) / 2.0).asRpm, delta)
    }
}
