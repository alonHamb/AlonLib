package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LengthTest {

    private val delta = 1e-6

    @Test
    fun `fromMeters round-trips through every unit`() {
        val length = Length.fromMeters(1.0)
        assertEquals(1.0, length.asMeters, delta)
        assertEquals(100.0, length.asCentimeters, delta)
        assertEquals(1000.0, length.asMillimeters, delta)
        assertEquals(3.28084, length.asFeet, 1e-4)
        assertEquals(39.3700787402, length.asInches, delta)
    }

    @Test
    fun `fromCentimeters converts correctly`() {
        assertEquals(1.0, Length.fromCentimeters(100.0).asMeters, delta)
    }

    @Test
    fun `fromMillimeters converts correctly`() {
        assertEquals(1.0, Length.fromMillimeters(1000.0).asMeters, delta)
    }

    @Test
    fun `fromFeet converts correctly`() {
        assertEquals(1.0, Length.fromFeet(3.28084).asMeters, 1e-4)
    }

    @Test
    fun `fromInches converts correctly`() {
        assertEquals(1.0, Length.fromInches(39.3700787402).asMeters, delta)
    }

    @Test
    fun `primary constructor with Meters unit stores the value directly`() {
        assertEquals(2.5, Length(2.5, Length.Unit.Meters).asMeters, delta)
    }

    @Test
    fun `primary constructor with Centimeters unit divides by 100`() {
        assertEquals(2.5, Length(250, Length.Unit.Centimeters).asMeters, delta)
    }

    @Test
    fun `primary constructor with Millimeters unit divides by 1000`() {
        assertEquals(2.5, Length(2500, Length.Unit.Millimeters).asMeters, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Length.fromMeters(Double.NaN).asMeters, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Length.fromMeters(Double.POSITIVE_INFINITY).asMeters, delta)
        assertEquals(0.0, Length.fromMeters(Double.NEGATIVE_INFINITY).asMeters, delta)
    }

    @Test
    fun `toString reports meters`() {
        assertEquals("Meters(1.5)", Length.fromMeters(1.5).toString())
    }

    @Test
    fun `compareTo orders by meters`() {
        assertTrue(Length.fromMeters(1.0) < Length.fromMeters(2.0))
        assertTrue(Length.fromMeters(2.0) > Length.fromMeters(1.0))
        assertEquals(0, Length.fromMeters(1.0).compareTo(Length.fromCentimeters(100.0)))
    }

    @Test
    fun `plus and minus operate in meters`() {
        val sum = Length.fromMeters(1.0) + Length.fromCentimeters(50.0)
        assertEquals(1.5, sum.asMeters, delta)

        val difference = Length.fromMeters(1.0) - Length.fromCentimeters(50.0)
        assertEquals(0.5, difference.asMeters, delta)
    }

    @Test
    fun `times and div operate on the raw meters values`() {
        val product = Length.fromMeters(2.0) * Length.fromMeters(3.0)
        assertEquals(6.0, product.asMeters, delta)

        val quotient = Length.fromMeters(6.0) / Length.fromMeters(2.0)
        assertEquals(3.0, quotient.asMeters, delta)
    }
}
