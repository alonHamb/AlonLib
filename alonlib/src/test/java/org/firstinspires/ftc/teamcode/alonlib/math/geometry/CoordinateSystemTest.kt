package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import org.junit.Assert.assertEquals
import org.junit.Test

class CoordinateSystemTest {

    private val delta = 1e-9

    @Test
    fun `NWU is the identity coordinate system (it's what the rest of the package already uses)`() {
        val v = Translation3d(1.0, 2.0, 3.0)
        val converted = CoordinateSystem.convert(v, CoordinateSystem.NWU(), CoordinateSystem.NWU())
        assertEquals(v.x, converted.x, delta)
        assertEquals(v.y, converted.y, delta)
        assertEquals(v.z, converted.z, delta)
    }

    @Test
    fun `converting 1 unit north from NWU to EDN lands on EDN's +Z axis`() {
        // EDN's +Z axis is defined to point north, so "north" should convert to (0, 0, 1).
        val north = Translation3d(1.0, 0.0, 0.0)
        val converted = CoordinateSystem.convert(north, CoordinateSystem.NWU(), CoordinateSystem.EDN())
        assertEquals(0.0, converted.x, delta)
        assertEquals(0.0, converted.y, delta)
        assertEquals(1.0, converted.z, delta)
    }

    @Test
    fun `converting 1 unit west from NWU to NED lands on NED's -Y axis`() {
        // NED's +Y axis is east, so west is -Y.
        val west = Translation3d(0.0, 1.0, 0.0)
        val converted = CoordinateSystem.convert(west, CoordinateSystem.NWU(), CoordinateSystem.NED())
        assertEquals(0.0, converted.x, delta)
        assertEquals(-1.0, converted.y, delta)
        assertEquals(0.0, converted.z, delta)
    }

    @Test
    fun `converting there and back is the identity`() {
        val v = Translation3d(3.0, -2.0, 5.0)
        val roundTripped = CoordinateSystem.convert(
            CoordinateSystem.convert(v, CoordinateSystem.NWU(), CoordinateSystem.EDN()),
            CoordinateSystem.EDN(),
            CoordinateSystem.NWU(),
        )
        assertEquals(v.x, roundTripped.x, delta)
        assertEquals(v.y, roundTripped.y, delta)
        assertEquals(v.z, roundTripped.z, delta)
    }

    @Test
    fun `rotation conversion between the same system is the identity`() {
        val r = Rotation3d.fromDegrees(10.0, 20.0, 30.0)
        assertEquals(r, CoordinateSystem.convert(r, CoordinateSystem.NWU(), CoordinateSystem.NWU()))
    }
}
