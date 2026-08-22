package org.firstinspires.ftc.teamcode.alonlib.math.filters.movingwindowfilters

import org.junit.Assert.assertEquals
import org.junit.Test

class MovingMedianFilterTest {

    @Test
    fun `takes the median of fewer samples than the window while it's still filling up`() {
        val filter = MovingMedianFilter(window = 5)

        assertEquals(10.0, filter.calculate(10.0), 1e-9)
        assertEquals(15.0, filter.calculate(20.0), 1e-9) // median(20, 10)
    }

    @Test
    fun `outliers only move the median once they're a majority of the window, and stop once evicted`() {
        val filter = MovingMedianFilter(window = 3)

        filter.calculate(1.0)
        filter.calculate(1.0)
        filter.calculate(1.0) // window [1, 1, 1]

        // A single 100 among two 1s doesn't move the median.
        assertEquals(1.0, filter.calculate(100.0), 1e-9) // window [100, 1, 1]
        // A second 100 makes outliers the majority (2 of 3).
        assertEquals(100.0, filter.calculate(100.0), 1e-9) // window [100, 100, 1]

        // Feeding 1s back in: the median only drops once the outliers are no longer the majority.
        assertEquals(100.0, filter.calculate(1.0), 1e-9) // window [1, 100, 100]
        assertEquals(1.0, filter.calculate(1.0), 1e-9) // window [1, 1, 100] -- back to normal
    }

    @Test
    fun `clear discards all previous samples`() {
        val filter = MovingMedianFilter(window = 3)
        filter.calculate(10.0)
        filter.calculate(1000.0)

        filter.clear()

        assertEquals(5.0, filter.calculate(5.0), 1e-9)
    }

    @Test
    fun `reset with a single value fills the whole window with it`() {
        val filter = MovingMedianFilter(window = 3)

        filter.reset(7.0)

        assertEquals(7.0, filter.calculate(7.0), 1e-9)
    }

    @Test
    fun `reset with a DoubleArray seeds the samples used by the next calculate`() {
        val filter = MovingMedianFilter(window = 5)

        filter.reset(doubleArrayOf(1.0, 2.0, 3.0))
        val result = filter.calculate(100.0) // [100, 1, 2, 3] -> sorted [1, 2, 3, 100], median 2.5

        assertEquals(2.5, result, 1e-9)
    }

    @Test
    fun `setting the window to a non-positive value clamps it to zero`() {
        val filter = MovingMedianFilter(window = 3)
        filter.window = 0
        assertEquals(0, filter.window)
    }
}
