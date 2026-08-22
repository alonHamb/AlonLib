package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TorqueTest {

    private val delta = 1e-3

    @Test
    fun `fromNewtonMeters round-trips through every unit`() {
        val torque = Torque.fromNewtonMeters(Torque.NEWTON_METERS_PER_POUND_FOOT)
        assertEquals(Torque.NEWTON_METERS_PER_POUND_FOOT, torque.asNewtonMeters, delta)
        assertEquals(1.0, torque.asPoundFeet, delta)
        assertEquals(12.0, torque.asPoundInches, delta)
        assertEquals(12.0 * 16.0, torque.asOunceInches, delta)
    }

    @Test
    fun `fromPoundFeet converts correctly`() {
        assertEquals(1.0, Torque.fromPoundFeet(1.0).asPoundFeet, delta)
    }

    @Test
    fun `fromPoundInches converts correctly`() {
        assertEquals(1.0, Torque.fromPoundInches(12.0).asPoundFeet, delta)
    }

    @Test
    fun `fromOunceInches converts correctly`() {
        assertEquals(1.0, Torque.fromOunceInches(12.0 * 16.0).asPoundFeet, delta)
    }

    @Test
    fun `fromKilogramCentimeters converts correctly`() {
        // 1 kgf*cm applied at 100cm (1m) leverage == 1 kgf*m of torque == g newton-meters.
        assertEquals(LinearAcceleration.STANDARD_GRAVITY, Torque.fromKilogramCentimeters(100.0).asNewtonMeters, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Torque.fromNewtonMeters(Double.NaN).asNewtonMeters, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Torque.fromNewtonMeters(Double.POSITIVE_INFINITY).asNewtonMeters, delta)
        assertEquals(0.0, Torque.fromNewtonMeters(Double.NEGATIVE_INFINITY).asNewtonMeters, delta)
    }

    @Test
    fun `toString reports newton-meters`() {
        assertEquals("NewtonMeters(1.5)", Torque.fromNewtonMeters(1.5).toString())
    }

    @Test
    fun `compareTo orders by newton-meters`() {
        assertTrue(Torque.fromNewtonMeters(1.0) < Torque.fromNewtonMeters(2.0))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in newton-meters`() {
        assertEquals(3.0, (Torque.fromNewtonMeters(1.0) + Torque.fromNewtonMeters(2.0)).asNewtonMeters, delta)
        assertEquals(1.0, (Torque.fromNewtonMeters(3.0) - Torque.fromNewtonMeters(2.0)).asNewtonMeters, delta)
        assertEquals(2.0, (Torque.fromNewtonMeters(1.0) * 2.0).asNewtonMeters, delta)
        assertEquals(0.5, (Torque.fromNewtonMeters(1.0) / 2.0).asNewtonMeters, delta)
        assertEquals(-1.0, (-Torque.fromNewtonMeters(1.0)).asNewtonMeters, delta)
    }
}
