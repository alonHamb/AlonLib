package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.junit.Assert.assertEquals
import org.junit.Test

class DifferentialDriveOdometryTest {

    private val delta = 1e-6

    @Test
    fun `driving both encoders forward equally moves straight along the initial heading`() {
        val odometry = DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0)
        val pose = odometry.update(Rotation2d.kZero, 2.0, 2.0)
        assertEquals(2.0, pose.x, delta)
        assertEquals(0.0, pose.y, delta)
    }

    @Test
    fun `odometry trusts the gyro angle over anything the encoders imply`() {
        // Even though the encoders alone would imply zero rotation (both moved forward equally),
        // the gyro says the robot turned 90deg -- the tracked heading should follow the gyro.
        val odometry = DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0)
        val pose = odometry.update(Rotation2d.fromDegrees(90.0), 1.0, 1.0)
        assertEquals(90.0, pose.rotation.degrees, delta)
    }

    @Test
    fun `resetPosition re-bases both the pose and the encoder deltas`() {
        val odometry = DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0)
        odometry.update(Rotation2d.kZero, 5.0, 5.0)

        odometry.resetPosition(Rotation2d.kZero, 5.0, 5.0, Pose2d(10.0, 10.0, Rotation2d.kZero))
        val pose = odometry.update(Rotation2d.kZero, 6.0, 6.0)

        assertEquals(11.0, pose.x, delta)
        assertEquals(10.0, pose.y, delta)
    }

    @Test
    fun `initial pose is returned before any update`() {
        val initial = Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45.0))
        val odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(45.0), 0.0, 0.0, initial)
        assertEquals(initial, odometry.pose)
    }

    @Test
    fun `gyro offset is preserved so a nonzero starting gyro reading still tracks correctly`() {
        // Robot starts facing 45deg per the initial pose, but the gyro itself reads 10deg at that
        // same instant (e.g. it wasn't zeroed) -- subsequent updates should be offset accordingly.
        val odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(10.0), 0.0, 0.0, Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0)))
        val pose = odometry.update(Rotation2d.fromDegrees(10.0), 0.0, 0.0)
        assertEquals(45.0, pose.rotation.degrees, delta)
    }
}
