package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import kotlin.math.abs
import kotlin.math.max

/** Wheel speeds for a differential (tank) drivetrain, in meters/second. */
class DifferentialDriveWheelSpeeds(var left: Double = 0.0, var right: Double = 0.0) {

    /**
     * Scales [left]/[right] down (preserving their ratio) if either exceeds
     * [attainableMaxSpeed] in magnitude.
     */
    fun desaturate(attainableMaxSpeed: Double) {
        val realMaxSpeed = max(abs(left), abs(right))
        if (realMaxSpeed > attainableMaxSpeed) {
            left = left / realMaxSpeed * attainableMaxSpeed
            right = right / realMaxSpeed * attainableMaxSpeed
        }
    }

    operator fun plus(other: DifferentialDriveWheelSpeeds) = DifferentialDriveWheelSpeeds(left + other.left, right + other.right)
    operator fun minus(other: DifferentialDriveWheelSpeeds) = DifferentialDriveWheelSpeeds(left - other.left, right - other.right)
    operator fun unaryMinus() = DifferentialDriveWheelSpeeds(-left, -right)
    operator fun times(scalar: Double) = DifferentialDriveWheelSpeeds(left * scalar, right * scalar)
    operator fun div(scalar: Double) = DifferentialDriveWheelSpeeds(left / scalar, right / scalar)

    override fun equals(other: Any?): Boolean {
        if (other !is DifferentialDriveWheelSpeeds) return false
        return left == other.left && right == other.right
    }

    override fun hashCode() = 31 * left.hashCode() + right.hashCode()

    override fun toString() = "DifferentialDriveWheelSpeeds(left=$left m/s, right=$right m/s)"
}
