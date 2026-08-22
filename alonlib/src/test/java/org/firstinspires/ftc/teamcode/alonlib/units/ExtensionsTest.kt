package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.sqrt

class ExtensionsTest {

    private val delta = 1e-6

    // -- Length ------------------------------------------------------------------------------

    @Test
    fun `Number to Length extensions match their factory functions`() {
        assertEquals(Length.fromMeters(2.0).asMeters, 2.0.meters.asMeters, delta)
        assertEquals(Length.fromCentimeters(2.0).asMeters, 2.0.centimeters.asMeters, delta)
        assertEquals(Length.fromMillimeters(2.0).asMeters, 2.0.millimeters.asMeters, delta)
        assertEquals(Length.fromFeet(2.0).asMeters, 2.0.feet.asMeters, delta)
        assertEquals(Length.fromInches(2.0).asMeters, 2.0.inches.asMeters, delta)
    }

    // -- AngularVelocity -----------------------------------------------------------------------

    @Test
    fun `Number to AngularVelocity extensions match their factory functions`() {
        assertEquals(AngularVelocity.fromRpm(2.0).asRpm, 2.0.rpm.asRpm, delta)
        assertEquals(AngularVelocity.fromRps(2.0).asRpm, 2.0.rps.asRpm, delta)
        assertEquals(AngularVelocity.fromRadPs(2.0).asRpm, 2.0.radPs.asRpm, delta)
        assertEquals(AngularVelocity.fromDegPs(2.0).asRpm, 2.0.degPs.asRpm, delta)
    }

    // -- Rotation2d ------------------------------------------------------------------------------

    @Test
    fun `Number to Rotation2d extensions`() {
        assertEquals(90.0, 90.0.degrees.degrees, delta)
        assertEquals(90.0, (Math.PI / 2.0).radians.degrees, delta)
        assertEquals(90.0, 0.25.rotations.degrees, delta)
    }

    @Test
    fun `Rotation2d absoluteValue drops the sign`() {
        assertEquals(90.0, (-90.0).degrees.absoluteValue.degrees, delta)
        assertEquals(90.0, 90.0.degrees.absoluteValue.degrees, delta)
    }

    @Test
    fun `Rotation2d rotations converts degrees to full turns`() {
        assertEquals(0.5, 180.0.degrees.rotations, delta)
        assertEquals(0.25, 90.0.degrees.rotations, delta)
    }

    @Test
    fun `Rotation2d normalizedDegrees wraps into the range 0 inclusive to 360 exclusive`() {
        // 190deg is itself stored canonicalized to -170deg (Rotation2d's own domain is (-180, 180]),
        // so this also exercises normalizedDegrees recovering the original positive angle.
        assertEquals(190.0, 190.0.degrees.normalizedDegrees, delta)
        assertEquals(170.0, 170.0.degrees.normalizedDegrees, delta)
        assertEquals(350.0, (-10.0).degrees.normalizedDegrees, delta)
    }

    @Test
    fun `Rotation2d normalizedRadians wraps into the range 0 inclusive to 2pi exclusive`() {
        assertEquals(Math.PI, 180.0.degrees.normalizedRadians, delta)
        assertEquals(Math.PI * 2.0 - degToRad(10.0), (-10.0).degrees.normalizedRadians, delta)
    }

    @Test
    fun `Rotation2d normalizedRotations wraps into the range 0 inclusive to 1 exclusive`() {
        assertEquals(0.5, 180.0.degrees.normalizedRotations, delta)
        assertEquals(190.0 / 360.0, 190.0.degrees.normalizedRotations, delta)
    }

    @Test
    fun `Rotation2d plus and minus add and subtract degrees`() {
        assertEquals(120.0, (90.0.degrees + 30.0.degrees).degrees, delta)
        assertEquals(60.0, (90.0.degrees - 30.0.degrees).degrees, delta)
    }

    @Test
    fun `Rotation2d times and div scale degrees`() {
        assertEquals(180.0, (90.0.degrees * 2.0).degrees, delta)
        assertEquals(45.0, (90.0.degrees / 2.0).degrees, delta)
    }

    @Test
    fun `Rotation2d rangeTo builds a range over degrees`() {
        val range = 10.0.degrees..20.0.degrees
        assertEquals(10.0, range.start, delta)
        assertEquals(20.0, range.endInclusive, delta)
    }

    @Test
    fun `Rotation2d compareTo orders by degrees`() {
        assertEquals(1, 20.0.degrees.compareTo(10.0.degrees))
        assertEquals(-1, 10.0.degrees.compareTo(20.0.degrees))
        assertEquals(0, 10.0.degrees.compareTo(10.0.degrees))
    }

    // -- Pose2d ------------------------------------------------------------------------------

    @Test
    fun `Pose2d distance and axis-distance extensions`() {
        val a = Pose2d(0.0, 0.0, Rotation2d(0.0))
        val b = Pose2d(3.0, 4.0, Rotation2d(0.0))

        assertEquals(3.0, a.xDistanceTo(b), delta)
        assertEquals(4.0, a.yDistanceTO(b), delta)
        assertEquals(5.0, a.distanceTo(b), delta)
        assertEquals(5.0, a.horizontalDistanceTo(b), delta)
        assertEquals(sqrt(3.0 * 3.0 + 4.0 * 4.0), a.distanceTo(b), delta)
    }

    @Test
    fun `Pose2d horizontalAngleTo points from this pose towards the other`() {
        val a = Pose2d(0.0, 0.0, Rotation2d(0.0))
        val b = Pose2d(1.0, 1.0, Rotation2d(0.0))

        assertEquals(45.0, a.horizontalAngleTo(b).degrees, delta)
    }
}
