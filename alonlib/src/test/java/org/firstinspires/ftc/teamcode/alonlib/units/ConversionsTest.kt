package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.junit.Assert.assertEquals
import org.junit.Test

class ConversionsTest {

    private val delta = 1e-6

    // -- Angles ----------------------------------------------------------------------------------

    @Test
    fun `degToRad and radToDeg are inverses`() {
        assertEquals(Math.PI, degToRad(180), delta)
        assertEquals(180.0, radToDeg(Math.PI), delta)
    }

    // -- Angular velocities ------------------------------------------------------------------------

    @Test
    fun `rpm conversions`() {
        assertEquals(1.0, rpmToRps(60), delta)
        assertEquals(Math.PI * 2.0, rpmToRadPs(60), delta)
        assertEquals(360.0, rpmToDegPs(60), delta)
    }

    @Test
    fun `rps conversions`() {
        assertEquals(60.0, rpsToRpm(1), delta)
        assertEquals(Math.PI * 2.0, rpsToRadPs(1), delta)
        assertEquals(360.0, rpsToDegPs(1), delta)
    }

    @Test
    fun `radPs conversions`() {
        assertEquals(60.0, radPsToRpm(Math.PI * 2.0), delta)
        assertEquals(1.0, radPsToRps(Math.PI * 2.0), delta)
        assertEquals(180.0, radPsToDegPs(Math.PI), delta)
    }

    @Test
    fun `degPs conversions`() {
        assertEquals(60.0, degPsToRpm(360), delta)
        assertEquals(1.0, degPsToRps(360), delta)
        assertEquals(Math.PI, degPsToRadPs(180), delta)
    }

    // -- Angular velocity to linear velocity ------------------------------------------------------

    @Test
    fun `rpmToMps uses the wheel's circumference`() {
        assertEquals(Math.PI * 2.0, rpmToMps(60, Length.fromMeters(1.0)), delta)
    }

    @Test
    fun `rpmToMps returns zero for a non-positive wheel radius`() {
        assertEquals(0.0, rpmToMps(60, Length.fromMeters(0.0)), delta)
        assertEquals(0.0, rpmToMps(60, Length.fromMeters(-1.0)), delta)
    }

    @Test
    fun `radPsToMps and degPsToMps delegate through rpmToMps`() {
        val wheelRadius = Length.fromMeters(1.0)
        assertEquals(rpmToMps(60, wheelRadius), radPsToMps(Math.PI * 2.0, wheelRadius), delta)
        assertEquals(rpmToMps(60, wheelRadius), degPsToMps(360, wheelRadius), delta)
    }

    @Test
    fun `mpsToRpm is the inverse of rpmToMps`() {
        val wheelRadius = Length.fromMeters(1.0)
        assertEquals(60.0, mpsToRpm(rpmToMps(60, wheelRadius), wheelRadius), delta)
    }

    @Test
    fun `mpsToRpm returns zero for a non-positive wheel radius`() {
        assertEquals(0.0, mpsToRpm(1.0, Length.fromMeters(0.0)), delta)
    }

    @Test
    fun `mpsToRadPs and mpsToDegPs delegate through mpsToRpm`() {
        val wheelRadius = Length.fromMeters(1.0)
        assertEquals(Math.PI * 2.0, mpsToRadPs(rpmToMps(60, wheelRadius), wheelRadius), delta)
        assertEquals(360.0, mpsToDegPs(rpmToMps(60, wheelRadius), wheelRadius), delta)
    }

    // -- Lengths -------------------------------------------------------------------------------

    @Test
    fun `meters, inches and feet conversions are consistent`() {
        assertEquals(INCHES_IN_METER, metersToInches(1), delta)
        assertEquals(1.0, inchesToMeters(INCHES_IN_METER), delta)
        assertEquals(1.0, inchesToFeet(12), delta)
        assertEquals(12.0, feetToInches(1), delta)
        assertEquals(1.0, metersToFeet(feetToMeters(1)), delta)
    }

    // -- Linear velocity unit conversions --------------------------------------------------------

    @Test
    fun `linear velocity conversions from meters per second`() {
        assertEquals(1000.0, mpsToMMps(1), delta)
        assertEquals(100.0, mpsToCMps(1), delta)
        assertEquals(3.6, mpsToKph(1), delta)
        assertEquals(39.37008, mpsToIps(1), delta)
        assertEquals(2.23693632, mpsToMph(1), delta)
        assertEquals(1.0, mmpsToMps(1000), delta)
    }

    // -- Alliance-relative pose ------------------------------------------------------------------

    @Test
    fun `matchPoseToAlliance passes a blue-alliance pose through unchanged`() {
        val pose = Pose2d(1.0, 2.0, 3.0.degrees)
        val result = matchPoseToAlliance(pose, Alliance.Blue)

        assertEquals(pose.x, result.x, delta)
        assertEquals(pose.y, result.y, delta)
        assertEquals(pose.rotation.degrees, result.rotation.degrees, delta)
    }

    @Test
    fun `matchPoseToAlliance mirrors x and adds 180 degrees for the red alliance`() {
        val pose = Pose2d(1.0, 2.0, 0.0.degrees)
        val result = matchPoseToAlliance(pose, Alliance.Red)

        assertEquals(DECODE_FIELD_LENGTH.asMeters - 1.0, result.x, delta)
        assertEquals(2.0, result.y, delta)
        assertEquals(180.0, result.rotation.degrees, delta)
    }
}
