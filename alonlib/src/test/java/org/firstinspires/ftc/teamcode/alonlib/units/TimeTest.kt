package org.firstinspires.ftc.teamcode.alonlib.units

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TimeTest {

    private val delta = 1e-6

    @Test
    fun `fromSeconds round-trips through every unit`() {
        val time = Time.fromSeconds(1.0)
        assertEquals(1.0, time.asSeconds, delta)
        assertEquals(1000.0, time.asMilliseconds, delta)
        assertEquals(1_000_000.0, time.asMicroseconds, delta)
        assertEquals(1_000_000_000.0, time.asNanoseconds, delta)
        assertEquals(1.0 / 60.0, time.asMinutes, delta)
        assertEquals(1.0 / 3600.0, time.asHours, delta)
    }

    @Test
    fun `fromMilliseconds converts correctly`() {
        assertEquals(1.0, Time.fromMilliseconds(1000.0).asSeconds, delta)
    }

    @Test
    fun `fromMicroseconds converts correctly`() {
        assertEquals(1.0, Time.fromMicroseconds(1_000_000.0).asSeconds, delta)
    }

    @Test
    fun `fromNanoseconds converts correctly`() {
        assertEquals(1.0, Time.fromNanoseconds(1_000_000_000.0).asSeconds, delta)
    }

    @Test
    fun `fromMinutes converts correctly`() {
        assertEquals(60.0, Time.fromMinutes(1.0).asSeconds, delta)
    }

    @Test
    fun `fromHours converts correctly`() {
        assertEquals(3600.0, Time.fromHours(1.0).asSeconds, delta)
    }

    @Test
    fun `NaN is coerced to zero`() {
        assertEquals(0.0, Time.fromSeconds(Double.NaN).asSeconds, delta)
    }

    @Test
    fun `infinite values are coerced to zero`() {
        assertEquals(0.0, Time.fromSeconds(Double.POSITIVE_INFINITY).asSeconds, delta)
        assertEquals(0.0, Time.fromSeconds(Double.NEGATIVE_INFINITY).asSeconds, delta)
    }

    @Test
    fun `toString reports seconds`() {
        assertEquals("Seconds(1.5)", Time.fromSeconds(1.5).toString())
    }

    @Test
    fun `compareTo orders by seconds`() {
        assertTrue(Time.fromSeconds(1.0) < Time.fromSeconds(2.0))
        assertEquals(0, Time.fromSeconds(60.0).compareTo(Time.fromMinutes(1.0)))
    }

    @Test
    fun `plus, minus, times, div and unaryMinus operate in seconds`() {
        assertEquals(1.5, (Time.fromSeconds(1.0) + Time.fromMilliseconds(500.0)).asSeconds, delta)
        assertEquals(0.5, (Time.fromSeconds(1.0) - Time.fromMilliseconds(500.0)).asSeconds, delta)
        assertEquals(2.0, (Time.fromSeconds(1.0) * 2.0).asSeconds, delta)
        assertEquals(0.5, (Time.fromSeconds(1.0) / 2.0).asSeconds, delta)
        assertEquals(-1.0, (-Time.fromSeconds(1.0)).asSeconds, delta)
    }
}
