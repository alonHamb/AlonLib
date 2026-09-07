package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d

/** Converts between [ChassisSpeeds] and per-side [DifferentialDriveWheelSpeeds]/[DifferentialDriveWheelPositions]. */
class DifferentialDriveKinematics(val trackWidthMeters: Double) :
        Kinematics<DifferentialDriveWheelSpeeds, DifferentialDriveWheelPositions> {

    override fun toChassisSpeeds(wheelSpeeds: DifferentialDriveWheelSpeeds) = ChassisSpeeds(
        (wheelSpeeds.left + wheelSpeeds.right) / 2.0,
        0.0,
        (wheelSpeeds.right - wheelSpeeds.left) / trackWidthMeters,
    )

    override fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds) = DifferentialDriveWheelSpeeds(
        chassisSpeeds.vx - trackWidthMeters / 2.0 * chassisSpeeds.omega,
        chassisSpeeds.vx + trackWidthMeters / 2.0 * chassisSpeeds.omega,
    )

    override fun toTwist2d(start: DifferentialDriveWheelPositions, end: DifferentialDriveWheelPositions) =
        toTwist2d(end.left - start.left, end.right - start.right)

    /** Forward kinematics from per-side distance deltas directly, for odometry. */
    fun toTwist2d(leftDistanceMeters: Double, rightDistanceMeters: Double) = Twist2d(
        (leftDistanceMeters + rightDistanceMeters) / 2.0,
        0.0,
        (rightDistanceMeters - leftDistanceMeters) / trackWidthMeters,
    )

    override fun interpolate(startValue: DifferentialDriveWheelPositions, endValue: DifferentialDriveWheelPositions, t: Double) =
        startValue.interpolate(endValue, t)
}
