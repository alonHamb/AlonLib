package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import kotlin.math.abs
import kotlin.math.max

/** Dead-wheel speeds for a [MecanumOdoKinematics] setup: left/right plus a strafing center wheel, in meters/second. */
class OdoWheelSpeeds(var left: Double = 0.0, var right: Double = 0.0, var center: Double = 0.0) {

    /** Scales all three speeds down (preserving their ratios) if any exceeds [attainableMaxSpeed] in magnitude. */
    fun normalize(attainableMaxSpeed: Double) {
        val realMaxSpeed = max(abs(left), abs(right))
        if (realMaxSpeed > attainableMaxSpeed) {
            left = left / realMaxSpeed * attainableMaxSpeed
            right = right / realMaxSpeed * attainableMaxSpeed
            center = center / realMaxSpeed * attainableMaxSpeed
        }
    }

    override fun toString() = "OdoWheelSpeeds(left=$left m/s, right=$right m/s, center=$center m/s)"
}
