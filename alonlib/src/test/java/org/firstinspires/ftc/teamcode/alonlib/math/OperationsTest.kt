package org.firstinspires.ftc.teamcode.alonlib.math

import org.junit.Assert.assertEquals
import org.junit.Test

class OperationsTest {

    // -- simpleDeadband ----------------------------------------------------------------------

    @Test
    fun `simpleDeadband zeroes values below the deadband`() {
        assertEquals(0.0, simpleDeadband(0.05, 0.1), 1e-9)
        assertEquals(0.0, simpleDeadband(-0.05, 0.1), 1e-9)
    }

    @Test
    fun `simpleDeadband passes through values at or above the deadband unchanged`() {
        assertEquals(0.1, simpleDeadband(0.1, 0.1), 1e-9)
        assertEquals(0.5, simpleDeadband(0.5, 0.1), 1e-9)
        assertEquals(-0.5, simpleDeadband(-0.5, 0.1), 1e-9)
    }

    @Test
    fun `simpleDeadband returns the value unchanged when deadband is negative`() {
        assertEquals(0.3, simpleDeadband(0.3, -0.1), 1e-9)
    }

    // -- continuousDeadband -------------------------------------------------------------------

    @Test
    fun `continuousDeadband matches the documented examples`() {
        assertEquals(0.0, continuousDeadband(0.05, 0.1), 1e-9)
        assertEquals(0.0, continuousDeadband(0.1, 0.1), 1e-9)
        assertEquals(0.44444, continuousDeadband(0.5, 0.1), 1e-4)
        assertEquals(1.0, continuousDeadband(1.0, 0.1), 1e-9)
    }

    @Test
    fun `continuousDeadband mirrors the mapping for negative values`() {
        assertEquals(0.0, continuousDeadband(-0.1, 0.1), 1e-9)
        assertEquals(-0.44444, continuousDeadband(-0.5, 0.1), 1e-4)
        assertEquals(-1.0, continuousDeadband(-1.0, 0.1), 1e-9)
    }

    @Test
    fun `continuousDeadband returns the value unchanged when deadband is out of bounds`() {
        assertEquals(0.5, continuousDeadband(0.5, 1.5), 1e-9)
        assertEquals(0.5, continuousDeadband(0.5, -0.1), 1e-9)
    }

    @Test
    fun `continuousDeadband returns the value unchanged when value is out of bounds`() {
        assertEquals(1.5, continuousDeadband(1.5, 0.1), 1e-9)
        assertEquals(-1.5, continuousDeadband(-1.5, 0.1), 1e-9)
    }

    // -- clamp ---------------------------------------------------------------------------------

    @Test
    fun `clamp coerces values into the given range`() {
        assertEquals(5.0, clamp(5.0, 0.0, 10.0), 1e-9)
        assertEquals(0.0, clamp(-5.0, 0.0, 10.0), 1e-9)
        assertEquals(10.0, clamp(15.0, 0.0, 10.0), 1e-9)
    }

    @Test
    fun `clamp returns zero when min is greater than max`() {
        assertEquals(0.0, clamp(5.0, 10.0, 0.0), 1e-9)
    }

    // -- mapRange (Double) -----------------------------------------------------------------------

    @Test
    fun `mapRange linearly maps a value between ranges`() {
        assertEquals(50.0, mapRange(5.0, 0.0, 10.0, 0.0, 100.0), 1e-9)
        assertEquals(0.0, mapRange(0.0, 0.0, 10.0, 0.0, 100.0), 1e-9)
        assertEquals(100.0, mapRange(10.0, 0.0, 10.0, 0.0, 100.0), 1e-9)
    }

    @Test
    fun `mapRange extrapolates outside the start range`() {
        assertEquals(-50.0, mapRange(-5.0, 0.0, 10.0, 0.0, 100.0), 1e-9)
        assertEquals(150.0, mapRange(15.0, 0.0, 10.0, 0.0, 100.0), 1e-9)
    }

    @Test
    fun `mapRange does not support an inverted end range -- endMin must be less than endMax`() {
        // Attempting to invert direction via endMin > endMax hits the same guard as
        // `mapRange returns the value unchanged when endMin is not less than endMax` below, and
        // returns the input unchanged rather than actually inverting it.
        assertEquals(0.25, mapRange(0.25, 0.0, 1.0, 100.0, 0.0), 1e-9)
    }

    @Test
    fun `mapRange returns the value unchanged when startMin is not less than startMax`() {
        assertEquals(5.0, mapRange(5.0, 10.0, 10.0, 0.0, 100.0), 1e-9)
        assertEquals(5.0, mapRange(5.0, 20.0, 10.0, 0.0, 100.0), 1e-9)
    }

    @Test
    fun `mapRange returns the value unchanged when endMin is not less than endMax`() {
        assertEquals(5.0, mapRange(5.0, 0.0, 10.0, 100.0, 100.0), 1e-9)
        assertEquals(5.0, mapRange(5.0, 0.0, 10.0, 100.0, 50.0), 1e-9)
    }

    // -- mapRange (Int) --------------------------------------------------------------------------

    @Test
    fun `mapRange for Int truncates the mapped Double result`() {
        assertEquals(50, mapRange(5, 0, 10, 0, 100))
        // 33.33... in Double truncates towards zero when converted to Int.
        assertEquals(33, mapRange(1, 0, 3, 0, 100))
    }

    // -- interpolate -----------------------------------------------------------------------------

    @Test
    fun `interpolate at t=0 returns startValue and at t=1 returns endValue`() {
        assertEquals(0.0, interpolate(0.0, 10.0, 0.0), 1e-9)
        assertEquals(10.0, interpolate(0.0, 10.0, 1.0), 1e-9)
    }

    @Test
    fun `interpolate at t=0,5 returns the midpoint`() {
        assertEquals(5.0, interpolate(0.0, 10.0, 0.5), 1e-9)
    }

    @Test
    fun `interpolate extrapolates outside t=0 to t=1`() {
        assertEquals(-10.0, interpolate(0.0, 10.0, -1.0), 1e-9)
        assertEquals(20.0, interpolate(0.0, 10.0, 2.0), 1e-9)
    }

    // -- inputModulus ------------------------------------------------------------------------------

    @Test
    fun `inputModulus wraps a value above the range back into it`() {
        assertEquals(-170.0, inputModulus(190.0, -180.0, 180.0), 1e-9)
    }

    @Test
    fun `inputModulus wraps a value below the range back into it`() {
        assertEquals(170.0, inputModulus(-190.0, -180.0, 180.0), 1e-9)
    }

    @Test
    fun `inputModulus leaves a value already inside the range unchanged`() {
        assertEquals(90.0, inputModulus(90.0, -180.0, 180.0), 1e-9)
    }

    @Test
    fun `inputModulus returns the value unchanged when minimumInput is not less than maximumInput`() {
        assertEquals(5.0, inputModulus(5.0, 10.0, 0.0), 1e-9)
    }

    // -- angleModulus --------------------------------------------------------------------------

    @Test
    fun `angleModulus wraps radians into the range -pi to pi`() {
        assertEquals(-Math.PI / 2.0, angleModulus(Math.PI * 1.5), 1e-9)
        assertEquals(Math.PI / 2.0, angleModulus(-Math.PI * 1.5), 1e-9)
        assertEquals(0.0, angleModulus(Math.PI * 2.0), 1e-9)
    }

    // -- isNear ------------------------------------------------------------------------------------

    @Test
    fun `isNear (plain) is true within tolerance and false outside it`() {
        assertEquals(true, isNear(5.0, 5.5, 1.0))
        assertEquals(false, isNear(5.0, 7.0, 1.0))
    }

    @Test
    fun `isNear (plain) returns false when tolerance is negative`() {
        assertEquals(false, isNear(5.0, 5.0, -1.0))
    }

    @Test
    fun `isNear (wraparound) treats values across the wrap point as close`() {
        // 179deg and -179deg are only 2deg apart once wrapped.
        assertEquals(true, isNear(179.0, -179.0, 5.0, -180.0, 180.0))
        assertEquals(false, isNear(179.0, -179.0, 1.0, -180.0, 180.0))
    }

    @Test
    fun `isNear (wraparound) returns false when tolerance is negative`() {
        assertEquals(false, isNear(0.0, 0.0, -1.0, -180.0, 180.0))
    }

    // -- median --------------------------------------------------------------------------------

    @Test
    fun `median of an odd-sized collection is the middle element`() {
        assertEquals(3.0, median(listOf(5.0, 1.0, 3.0, 4.0, 2.0)), 1e-9)
    }

    @Test
    fun `median of an even-sized collection averages the two middle elements`() {
        assertEquals(2.5, median(listOf(1.0, 2.0, 3.0, 4.0)), 1e-9)
    }

    @Test
    fun `median of exactly two elements averages them`() {
        assertEquals(3.0, median(listOf(1.0, 5.0)), 1e-9)
    }

    @Test
    fun `median works from a DoubleArray`() {
        assertEquals(2.0, median(doubleArrayOf(3.0, 1.0, 2.0)), 1e-9)
    }

    @Test
    fun `median works from an Array of Double`() {
        assertEquals(2.0, median(arrayOf(3.0, 1.0, 2.0)), 1e-9)
    }

    @Test
    fun `median of a single element is itself`() {
        assertEquals(7.0, median(listOf(7.0)), 1e-9)
    }
}
