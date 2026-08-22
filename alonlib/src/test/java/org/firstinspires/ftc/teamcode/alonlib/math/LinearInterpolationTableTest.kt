package org.firstinspires.ftc.teamcode.alonlib.math

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class LinearInterpolationTableTest {

    private val table = LinearInterpolationTable(
        0.0 to 0.0,
        10.0 to 100.0,
        20.0 to 300.0,
    )

    @Test
    fun `interpolates linearly between two points`() {
        assertEquals(50.0, table.getOutputFor(5.0), 1e-9)
    }

    @Test
    fun `returns the exact output for an input that matches a point`() {
        assertEquals(0.0, table.getOutputFor(0.0), 1e-9)
        assertEquals(100.0, table.getOutputFor(10.0), 1e-9)
        assertEquals(300.0, table.getOutputFor(20.0), 1e-9)
    }

    @Test
    fun `extrapolates using the first segment's slope below the minimum input`() {
        assertEquals(-50.0, table.getOutputFor(-5.0), 1e-9)
    }

    @Test
    fun `extrapolates using the last segment's slope above the maximum input`() {
        // Slope of the last segment is (300-100)/(20-10) = 20 per unit input.
        assertEquals(400.0, table.getOutputFor(25.0), 1e-9)
    }

    @Test
    fun `interpolates within the second segment`() {
        assertEquals(200.0, table.getOutputFor(15.0), 1e-9)
    }

    @Test
    fun `firsts and seconds expose the raw table columns in insertion order`() {
        assertArrayEquals(doubleArrayOf(0.0, 10.0, 20.0), table.firsts, 1e-9)
        assertArrayEquals(doubleArrayOf(0.0, 100.0, 300.0), table.seconds, 1e-9)
    }

    @Test
    fun `works with just two points`() {
        val twoPointTable = LinearInterpolationTable(0.0 to 0.0, 1.0 to 10.0)
        assertEquals(5.0, twoPointTable.getOutputFor(0.5), 1e-9)
    }
}
