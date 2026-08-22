package org.firstinspires.ftc.teamcode.alonlib.purepursuit

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PurePursuitUtilTest {

    private val delta = 1e-6

    @Test
    fun `angleWrap keeps angles within -pi to pi`() {
        assertEquals(0.0, PurePursuitUtil.angleWrap(2 * Math.PI), delta)
        assertEquals(Math.PI - 0.1, PurePursuitUtil.angleWrap(-Math.PI - 0.1), 1e-3)
    }

    @Test
    fun `lineCircleIntersection finds both crossings of a horizontal line through a circle`() {
        val points = PurePursuitUtil.lineCircleIntersection(
            Translation2d(0.0, 0.0), 1.0, Translation2d(-2.0, 0.0), Translation2d(2.0, 0.0),
        )
        assertEquals(2, points.size)
        assertTrue(points.any { kotlin.math.abs(it.x - 1.0) < delta })
        assertTrue(points.any { kotlin.math.abs(it.x + 1.0) < delta })
    }

    @Test
    fun `lineCircleIntersection returns nothing when the line misses the circle`() {
        val points = PurePursuitUtil.lineCircleIntersection(
            Translation2d(0.0, 5.0), 1.0, Translation2d(-2.0, 0.0), Translation2d(2.0, 0.0),
        )
        assertTrue(points.isEmpty())
    }

    @Test
    fun `lineCircleIntersection excludes crossings outside the line segment`() {
        // Circle centered past the end of the segment -- the math-line crosses it, but the segment doesn't reach that far.
        val points = PurePursuitUtil.lineCircleIntersection(
            Translation2d(5.0, 0.0), 1.0, Translation2d(-1.0, 0.0), Translation2d(1.0, 0.0),
        )
        assertTrue(points.isEmpty())
    }

    @Test
    fun `moveToPosition drives straight toward a target directly ahead`() {
        // Facing +x already (heading 0), target directly ahead along +x: this convention puts that
        // motion entirely in powers[0] ("x power" per the javadoc), with powers[1] ("y power") at zero.
        val powers = PurePursuitUtil.moveToPosition(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, false)
        assertEquals(1.0, powers[0], delta)
        assertEquals(0.0, powers[1], delta)
    }

    @Test
    fun `moveToPosition with turnOnly ignores position and only returns a turn component`() {
        val powers = PurePursuitUtil.moveToPosition(0.0, 0.0, 0.0, 100.0, 100.0, Math.PI / 2, true)
        assertEquals(0.0, powers[0], delta)
        assertEquals(0.0, powers[1], delta)
    }
}
