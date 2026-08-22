package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.junit.Assert.assertEquals
import org.junit.Test

class SwerveDriveKinematicsTest {

    private val delta = 1e-6

    private val kinematics = SwerveDriveKinematics(
        Translation2d(0.5, 0.5),
        Translation2d(0.5, -0.5),
        Translation2d(-0.5, 0.5),
        Translation2d(-0.5, -0.5),
    )

    @Test
    fun `zero chassis speed keeps each module's last heading instead of snapping to zero`() {
        kinematics.toSwerveModuleStates(ChassisSpeeds(1.0, 0.0, 0.0))
        val states = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 0.0))
        for (state in states) {
            assertEquals(0.0, state.speedMetersPerSecond, delta)
            assertEquals(0.0, state.angle.degrees, delta)
        }
    }

    @Test
    fun `forward is the inverse of inverse kinematics for a straight-line move`() {
        val chassisSpeeds = ChassisSpeeds(1.0, 0.0, 0.0)
        val states = kinematics.toSwerveModuleStates(chassisSpeeds)
        val roundTripped = kinematics.toChassisSpeeds(states)

        assertEquals(chassisSpeeds.vx, roundTripped.vx, delta)
        assertEquals(chassisSpeeds.vy, roundTripped.vy, delta)
        assertEquals(chassisSpeeds.omega, roundTripped.omega, delta)
    }

    @Test
    fun `forward is the inverse of inverse kinematics for a combined move`() {
        val chassisSpeeds = ChassisSpeeds(0.6, -0.4, 0.8)
        val roundTripped = kinematics.toChassisSpeeds(kinematics.toSwerveModuleStates(chassisSpeeds))

        assertEquals(chassisSpeeds.vx, roundTripped.vx, delta)
        assertEquals(chassisSpeeds.vy, roundTripped.vy, delta)
        assertEquals(chassisSpeeds.omega, roundTripped.omega, delta)
    }

    @Test
    fun `pure rotation makes every module point tangent to the rotation center`() {
        // Front-left at (0.5, 0.5): tangent to a CCW rotation about the origin points at 135deg.
        val states = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 1.0))
        assertEquals(135.0, states[0].angle.degrees, delta)
    }

    @Test
    fun `desaturateWheelSpeeds scales every module down preserving their ratios`() {
        val states = arrayOf(
            SwerveModuleState(4.0, Rotation2d.kZero),
            SwerveModuleState(2.0, Rotation2d.kZero),
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 2.0)
        assertEquals(2.0, states[0].speedMetersPerSecond, delta)
        assertEquals(1.0, states[1].speedMetersPerSecond, delta)
    }

    @Test
    fun `SwerveModuleState optimize reverses speed and flips angle for a greater-than-90deg change`() {
        val state = SwerveModuleState(1.0, Rotation2d.fromDegrees(180.0))
        state.optimize(Rotation2d.kZero)
        assertEquals(-1.0, state.speedMetersPerSecond, delta)
        assertEquals(0.0, state.angle.degrees, delta)
    }

    @Test
    fun `SwerveModuleState optimize leaves a within-90deg change unchanged`() {
        val state = SwerveModuleState(1.0, Rotation2d.fromDegrees(45.0))
        state.optimize(Rotation2d.kZero)
        assertEquals(1.0, state.speedMetersPerSecond, delta)
        assertEquals(45.0, state.angle.degrees, delta)
    }
}
