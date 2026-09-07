package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d

/**
 * Converts between [ChassisSpeeds] and a drivetrain's own [WheelSpeeds]/[WheelPositions] shape
 * (e.g. left/right for a differential drive, four corners for mecanum/swerve). Backs the generic
 * [Odometry] base class.
 */
interface Kinematics<WheelSpeeds, WheelPositions> {
    fun toChassisSpeeds(wheelSpeeds: WheelSpeeds): ChassisSpeeds
    fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds): WheelSpeeds

    /** The pose delta between [start] and [end]'s wheel positions, for one [Odometry.update] tick. */
    fun toTwist2d(start: WheelPositions, end: WheelPositions): Twist2d

    fun interpolate(startValue: WheelPositions, endValue: WheelPositions, t: Double): WheelPositions
}
