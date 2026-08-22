package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class ChassisSpeedsTest {

    private val delta = 1e-6

    @Test
    fun `discretize of a pure straight-line move is unchanged`() {
        val discretized = ChassisSpeeds.discretize(1.0, 0.0, 0.0, 0.02)
        assertEquals(1.0, discretized.vx, delta)
        assertEquals(0.0, discretized.vy, delta)
        assertEquals(0.0, discretized.omega, delta)
    }

    @Test
    fun `discretize of a pure rotation is unchanged`() {
        val discretized = ChassisSpeeds.discretize(0.0, 0.0, 1.0, 0.02)
        assertEquals(0.0, discretized.vx, delta)
        assertEquals(0.0, discretized.vy, delta)
        assertEquals(1.0, discretized.omega, delta)
    }

    @Test
    fun `discretize introduces a small nonzero vy skew when translating and rotating together`() {
        val discretized = ChassisSpeeds.discretize(1.0, 0.0, 1.0, 0.02)
        assertTrue(discretized.vy != 0.0)
        assertTrue(abs(discretized.vy) < 0.02)
    }

    @Test
    fun `fromFieldRelativeSpeeds and fromRobotRelativeSpeeds are inverses`() {
        val robotAngle = Rotation2d.fromDegrees(30.0)
        val fieldRelative = ChassisSpeeds(1.0, 2.0, 0.5)
        val robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, robotAngle)
        val roundTripped = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, robotAngle)

        assertEquals(fieldRelative.vx, roundTripped.vx, delta)
        assertEquals(fieldRelative.vy, roundTripped.vy, delta)
        assertEquals(fieldRelative.omega, roundTripped.omega, delta)
    }

    @Test
    fun `robot facing 90deg moving field +x moves robot-relative -y`() {
        // Facing +90deg (i.e. towards field +y), field-relative +x (away from start) is to the
        // robot's right, i.e. robot-relative -y.
        val robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(1.0, 0.0, 0.0, Rotation2d.fromDegrees(90.0))
        assertEquals(0.0, robotRelative.vx, delta)
        assertEquals(-1.0, robotRelative.vy, delta)
    }

    @Test
    fun `plus, minus, times, div, unaryMinus operate component-wise`() {
        val a = ChassisSpeeds(1.0, 2.0, 3.0)
        val b = ChassisSpeeds(0.5, 0.5, 0.5)
        assertEquals(ChassisSpeeds(1.5, 2.5, 3.5), a + b)
        assertEquals(ChassisSpeeds(0.5, 1.5, 2.5), a - b)
        assertEquals(ChassisSpeeds(2.0, 4.0, 6.0), a * 2.0)
        assertEquals(ChassisSpeeds(0.5, 1.0, 1.5), a / 2.0)
        assertEquals(ChassisSpeeds(-1.0, -2.0, -3.0), -a)
    }
}
