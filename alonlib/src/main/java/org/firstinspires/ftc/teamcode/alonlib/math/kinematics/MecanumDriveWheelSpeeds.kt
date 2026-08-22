package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import kotlin.math.abs

/** Wheel speeds for a mecanum drivetrain, in meters/second. */
class MecanumDriveWheelSpeeds(
    var frontLeft: Double = 0.0,
    var frontRight: Double = 0.0,
    var rearLeft: Double = 0.0,
    var rearRight: Double = 0.0,
) {

    /** Scales all four speeds down (preserving their ratios) if any exceeds [attainableMaxSpeed] in magnitude. */
    fun desaturate(attainableMaxSpeed: Double) {
        val realMaxSpeed = maxOf(abs(frontLeft), abs(frontRight), abs(rearLeft), abs(rearRight))
        if (realMaxSpeed > attainableMaxSpeed) {
            frontLeft = frontLeft / realMaxSpeed * attainableMaxSpeed
            frontRight = frontRight / realMaxSpeed * attainableMaxSpeed
            rearLeft = rearLeft / realMaxSpeed * attainableMaxSpeed
            rearRight = rearRight / realMaxSpeed * attainableMaxSpeed
        }
    }

    operator fun plus(other: MecanumDriveWheelSpeeds) =
        MecanumDriveWheelSpeeds(frontLeft + other.frontLeft, frontRight + other.frontRight, rearLeft + other.rearLeft, rearRight + other.rearRight)

    operator fun minus(other: MecanumDriveWheelSpeeds) =
        MecanumDriveWheelSpeeds(frontLeft - other.frontLeft, frontRight - other.frontRight, rearLeft - other.rearLeft, rearRight - other.rearRight)

    operator fun unaryMinus() = MecanumDriveWheelSpeeds(-frontLeft, -frontRight, -rearLeft, -rearRight)
    operator fun times(scalar: Double) = MecanumDriveWheelSpeeds(frontLeft * scalar, frontRight * scalar, rearLeft * scalar, rearRight * scalar)
    operator fun div(scalar: Double) = MecanumDriveWheelSpeeds(frontLeft / scalar, frontRight / scalar, rearLeft / scalar, rearRight / scalar)

    override fun equals(other: Any?): Boolean {
        if (other !is MecanumDriveWheelSpeeds) return false
        return frontLeft == other.frontLeft && frontRight == other.frontRight && rearLeft == other.rearLeft && rearRight == other.rearRight
    }

    override fun hashCode() = arrayOf(frontLeft, frontRight, rearLeft, rearRight).contentHashCode()

    override fun toString() = "MecanumDriveWheelSpeeds(frontLeft=$frontLeft, frontRight=$frontRight, rearLeft=$rearLeft, rearRight=$rearRight)"
}
