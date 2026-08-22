package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class InterpolatingTreeMapTest {

    private val delta = 1e-9

    // -- InterpolatingDoubleTreeMap ------------------------------------------------------------

    @Test
    fun `get on an empty map returns null`() {
        assertNull(InterpolatingDoubleTreeMap().get(5.0))
    }

    @Test
    fun `get on an exact key returns that key's value without interpolating`() {
        val table = InterpolatingDoubleTreeMap()
        table.put(1.0, 10.0)
        table.put(2.0, 20.0)
        assertEquals(10.0, table.get(1.0)!!, delta)
    }

    @Test
    fun `get between two keys linearly interpolates`() {
        val table = InterpolatingDoubleTreeMap()
        table.put(0.0, 0.0)
        table.put(10.0, 100.0)
        assertEquals(50.0, table.get(5.0)!!, delta)
        assertEquals(30.0, table.get(3.0)!!, delta)
    }

    @Test
    fun `get below the lowest key clamps to the lowest value`() {
        val table = InterpolatingDoubleTreeMap()
        table.put(0.0, 0.0)
        table.put(10.0, 100.0)
        assertEquals(0.0, table.get(-5.0)!!, delta)
    }

    @Test
    fun `get above the highest key clamps to the highest value`() {
        val table = InterpolatingDoubleTreeMap()
        table.put(0.0, 0.0)
        table.put(10.0, 100.0)
        assertEquals(100.0, table.get(15.0)!!, delta)
    }

    @Test
    fun `put overwrites an existing key`() {
        val table = InterpolatingDoubleTreeMap()
        table.put(1.0, 10.0)
        table.put(1.0, 20.0)
        assertEquals(20.0, table.get(1.0)!!, delta)
        assertEquals(1, table.size)
    }

    @Test
    fun `size and isEmpty track the number of entries`() {
        val table = InterpolatingDoubleTreeMap()
        assertTrue(table.isEmpty)
        table.put(1.0, 10.0)
        assertEquals(1, table.size)
        assertTrue(!table.isEmpty)
        table.clear()
        assertTrue(table.isEmpty)
    }

    // -- InverseInterpolator.forDouble -------------------------------------------------------

    @Test
    fun `InverseInterpolator forDouble finds the fraction between start and end`() {
        assertEquals(0.5, InverseInterpolator.forDouble.inverseInterpolate(0.0, 10.0, 5.0), delta)
        assertEquals(0.0, InverseInterpolator.forDouble.inverseInterpolate(0.0, 10.0, -5.0), delta)
        assertEquals(1.0, InverseInterpolator.forDouble.inverseInterpolate(0.0, 10.0, 15.0), delta)
    }

    @Test
    fun `InverseInterpolator forDouble returns zero when the range is empty or inverted`() {
        assertEquals(0.0, InverseInterpolator.forDouble.inverseInterpolate(5.0, 5.0, 5.0), delta)
        assertEquals(0.0, InverseInterpolator.forDouble.inverseInterpolate(10.0, 0.0, 5.0), delta)
    }

    // -- generic Interpolatable support --------------------------------------------------------

    private data class Fahrenheit(val degrees: Double) : Interpolatable<Fahrenheit> {
        override fun interpolate(endValue: Fahrenheit, t: Double) =
            Fahrenheit(degrees + (endValue.degrees - degrees) * t)
    }

    @Test
    fun `InterpolatingTreeMap works with a custom Interpolatable type`() {
        val table = InterpolatingTreeMap<Double, Fahrenheit>(
            InverseInterpolator.forDouble,
            Interpolator.forInterpolatable(),
        )
        table.put(0.0, Fahrenheit(32.0))
        table.put(10.0, Fahrenheit(212.0))

        assertEquals(122.0, table.get(5.0)!!.degrees, delta)
    }
}
