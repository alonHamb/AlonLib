package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.junit.Assert.assertEquals
import org.junit.Test

class MecanumDriveKinematicsTest {

    private val delta = 1e-6

    private val kinematics = MecanumDriveKinematics(
        frontLeftWheel = Translation2d(0.5, 0.5),
        frontRightWheel = Translation2d(0.5, -0.5),
        rearLeftWheel = Translation2d(-0.5, 0.5),
        rearRightWheel = Translation2d(-0.5, -0.5),
    )

    @Test
    fun `forward is the inverse of inverse kinematics for a straight-line move`() {
        val chassisSpeeds = ChassisSpeeds(1.0, 0.0, 0.0)
        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)
        val roundTripped = kinematics.toChassisSpeeds(wheelSpeeds)

        assertEquals(chassisSpeeds.vx, roundTripped.vx, delta)
        assertEquals(chassisSpeeds.vy, roundTripped.vy, delta)
        assertEquals(chassisSpeeds.omega, roundTripped.omega, delta)
    }

    @Test
    fun `forward is the inverse of inverse kinematics for a strafing move`() {
        val chassisSpeeds = ChassisSpeeds(0.0, 1.0, 0.0)
        val roundTripped = kinematics.toChassisSpeeds(kinematics.toWheelSpeeds(chassisSpeeds))

        assertEquals(chassisSpeeds.vx, roundTripped.vx, delta)
        assertEquals(chassisSpeeds.vy, roundTripped.vy, delta)
        assertEquals(chassisSpeeds.omega, roundTripped.omega, delta)
    }

    @Test
    fun `forward is the inverse of inverse kinematics for a rotating move`() {
        val chassisSpeeds = ChassisSpeeds(0.0, 0.0, 1.0)
        val roundTripped = kinematics.toChassisSpeeds(kinematics.toWheelSpeeds(chassisSpeeds))

        assertEquals(chassisSpeeds.vx, roundTripped.vx, delta)
        assertEquals(chassisSpeeds.vy, roundTripped.vy, delta)
        assertEquals(chassisSpeeds.omega, roundTripped.omega, delta)
    }

    @Test
    fun `forward is the inverse of inverse kinematics for a combined move`() {
        val chassisSpeeds = ChassisSpeeds(0.7, -0.3, 0.5)
        val roundTripped = kinematics.toChassisSpeeds(kinematics.toWheelSpeeds(chassisSpeeds))

        assertEquals(chassisSpeeds.vx, roundTripped.vx, delta)
        assertEquals(chassisSpeeds.vy, roundTripped.vy, delta)
        assertEquals(chassisSpeeds.omega, roundTripped.omega, delta)
    }

    @Test
    fun `pure rotation drives all four wheels at equal magnitude but alternating sign`() {
        val wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds(0.0, 0.0, 1.0))
        assertEquals(-wheelSpeeds.frontLeft, wheelSpeeds.frontRight, delta)
        assertEquals(wheelSpeeds.frontLeft, wheelSpeeds.rearLeft, delta)
        assertEquals(wheelSpeeds.frontRight, wheelSpeeds.rearRight, delta)
    }

    @Test
    fun `desaturate scales all four wheels down preserving their ratios`() {
        val speeds = MecanumDriveWheelSpeeds(4.0, 2.0, 1.0, 3.0)
        speeds.desaturate(2.0)
        assertEquals(2.0, speeds.frontLeft, delta)
        assertEquals(1.0, speeds.frontRight, delta)
        assertEquals(0.5, speeds.rearLeft, delta)
        assertEquals(1.5, speeds.rearRight, delta)
    }
}
