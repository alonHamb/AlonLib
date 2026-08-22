package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.junit.Assert.assertEquals
import org.junit.Test

class DifferentialDriveKinematicsTest {

    private val delta = 1e-9
    private val kinematics = DifferentialDriveKinematics(trackWidthMeters = 1.0)

    @Test
    fun `straight-line wheel speeds produce zero rotation`() {
        val chassisSpeeds = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(2.0, 2.0))
        assertEquals(2.0, chassisSpeeds.vx, delta)
        assertEquals(0.0, chassisSpeeds.omega, delta)
    }

    @Test
    fun `opposite wheel speeds produce pure rotation`() {
        val chassisSpeeds = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(-1.0, 1.0))
        assertEquals(0.0, chassisSpeeds.vx, delta)
        assertEquals(2.0, chassisSpeeds.omega, delta)
    }

    @Test
    fun `toWheelSpeeds is the inverse of toChassisSpeeds`() {
        val original = DifferentialDriveWheelSpeeds(1.5, -0.5)
        val chassisSpeeds = kinematics.toChassisSpeeds(original)
        val roundTripped = kinematics.toWheelSpeeds(chassisSpeeds)

        assertEquals(original.left, roundTripped.left, delta)
        assertEquals(original.right, roundTripped.right, delta)
    }

    @Test
    fun `toTwist2d matches toChassisSpeeds scaled by a unit time`() {
        val twist = kinematics.toTwist2d(2.0, 2.0)
        val chassisSpeeds = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(2.0, 2.0))
        assertEquals(chassisSpeeds.vx, twist.dx, delta)
        assertEquals(chassisSpeeds.omega, twist.dtheta, delta)
    }

    @Test
    fun `desaturate scales both sides down preserving their ratio`() {
        val speeds = DifferentialDriveWheelSpeeds(4.0, 2.0)
        speeds.desaturate(2.0)
        assertEquals(2.0, speeds.left, delta)
        assertEquals(1.0, speeds.right, delta)
    }

    @Test
    fun `desaturate does nothing when already within the limit`() {
        val speeds = DifferentialDriveWheelSpeeds(1.0, 0.5)
        speeds.desaturate(2.0)
        assertEquals(1.0, speeds.left, delta)
        assertEquals(0.5, speeds.right, delta)
    }
}
