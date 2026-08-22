package org.firstinspires.ftc.teamcode.alonlib.math.filters.movingwindowfilters

import org.junit.Assert.assertEquals
import org.junit.Test

class MovingAverageFilterTest {

    @Test
    fun `averages fewer samples than the window while it's still filling up`() {
        val filter = MovingAverageFilter(window = 3)

        assertEquals(2.0, filter.calculate(2.0), 1e-9)
        assertEquals(3.0, filter.calculate(4.0), 1e-9) // avg(4, 2)
    }

    @Test
    fun `averages only the most recent 'window' samples once full`() {
        val filter = MovingAverageFilter(window = 3)

        filter.calculate(1.0)
        filter.calculate(2.0)
        filter.calculate(3.0)
        // Window is now full at [3, 2, 1]; the next call should drop the oldest sample (1).
        val result = filter.calculate(10.0) // avg(10, 3, 2)
        assertEquals(5.0, result, 1e-9)
    }

    @Test
    fun `clear discards all previous samples`() {
        val filter = MovingAverageFilter(window = 3)
        filter.calculate(10.0)
        filter.calculate(20.0)

        filter.clear()

        assertEquals(5.0, filter.calculate(5.0), 1e-9)
    }

    @Test
    fun `reset with a DoubleArray seeds the samples used by the next calculate`() {
        val filter = MovingAverageFilter(window = 3)

        filter.reset(doubleArrayOf(10.0, 20.0))
        // Seeded samples are [10, 20]; calculate adds a new sample and, since the window (3) isn't
        // exceeded, keeps all three.
        val result = filter.calculate(30.0)
        assertEquals(20.0, result, 1e-9) // avg(30, 10, 20)
    }

    @Test
    fun `reset with a Collection delegates to the DoubleArray overload`() {
        val filter = MovingAverageFilter(window = 2)

        filter.reset(listOf(4.0, 6.0))
        // Seeded samples [4, 6] already fill the window of 2, so calculate first evicts the oldest
        // (6) before adding the new sample: avg(5, 4).
        assertEquals(4.5, filter.calculate(5.0), 1e-9)
    }

    @Test
    fun `reset with an Array of Double delegates to the DoubleArray overload`() {
        val filter = MovingAverageFilter(window = 3)

        filter.reset(arrayOf(1.0, 2.0, 3.0))
        // Seeded samples [1, 2, 3] already fill the window of 3, so calculate first evicts the
        // oldest (3) before adding the new sample: avg(2, 1, 2).
        assertEquals(5.0 / 3.0, filter.calculate(2.0), 1e-9)
    }

    @Test
    fun `reset with a single value fills the whole window with it`() {
        val filter = MovingAverageFilter(window = 4)

        filter.reset(5.0)

        // All 4 slots are 5.0; adding a new 5.0 (and dropping the oldest) keeps the average at 5.0.
        assertEquals(5.0, filter.calculate(5.0), 1e-9)
    }

    @Test
    fun `setting the window to a positive value updates it`() {
        val filter = MovingAverageFilter(window = 3)
        filter.window = 5
        assertEquals(5, filter.window)
    }

    @Test
    fun `setting the window to a non-positive value clamps it to zero`() {
        val filter = MovingAverageFilter(window = 3)
        filter.window = -1
        assertEquals(0, filter.window)
    }

    @Test
    fun `the constructor's window argument bypasses the property's own validation`() {
        // Kotlin property initializers assign the backing field directly rather than going through
        // a custom setter, so (unlike the `window = ...` setter above) this does NOT clamp to zero.
        val filter = MovingAverageFilter(window = -3)
        assertEquals(-3, filter.window)
    }
}
