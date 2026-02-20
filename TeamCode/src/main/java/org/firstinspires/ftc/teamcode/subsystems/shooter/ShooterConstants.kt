package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.hamosad1657.lib.math.PIDGains
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rangeTo
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

object ShooterConstants {
    // --- Ratios ---
    val HEADING_RATIO = 11.0 / 140.0

    // --- Mechanical limits ---
    val MAXIMUM_HEADING = 360.degrees
    val MINIMUM_HEADING = 0.degrees
    val MAXIMUM_ANGLE = 45.degrees
    val MINIMUM_ANGLE = 8.degrees
    val MAXIMUM_VELOCITY = 4700.rpm
    val MINIMUM_VELOCITY = 100.rpm
    val MAXIMUM_HEADING_ANGLE = 360.0.degrees
    val MINIMUM_HEADING_ANGLE = 0.0.degrees

    // --- PID parameters ---
    val VELOCITY_PID_GAINS = PIDGains(0.0, 0.0, 0.0, 0.0)
    val HEADING_PID_GAINS = PIDGains(0.0, 0.0, 0.0, 0.0)

    // --- tolerances ---
    val HEADING_TOLERANCE = 1.degrees
    val VELOCITY_TOLERANCE = 100.rpm

    // --- tag positions ---


    class ShooterState(angle: Rotation2d, heading: Rotation2d, velocity: AngularVelocity) {
        val angle: Rotation2d
        val velocity: AngularVelocity
        val heading: Rotation2d

        init {
            this.angle =
                if (angle.degrees in MINIMUM_ANGLE..MAXIMUM_ANGLE) angle
                else {
                    robotPrintError("Shooter angle out of bounds: ${angle.degrees}")
                    0.degrees
                }

            this.velocity =
                if (velocity in MINIMUM_VELOCITY..MAXIMUM_VELOCITY) velocity
                else {
                    robotPrintError("Shooter velocity out of bounds: ${velocity.asRpm}")
                    0.rpm
                }

            this.heading =
                if (heading.degrees in MINIMUM_HEADING_ANGLE..MAXIMUM_HEADING_ANGLE) heading
                else {
                    robotPrintError("heading angle out of bounds: ${heading.degrees}")
                    0.degrees
                }
        }

        companion object {
            // --- teleop ---
            val AT_GOAL = ShooterState(0.0.degrees, 0.0.degrees, 1500.rpm)
            val AT_FIELD_CENTER = ShooterState(45.degrees, 0.degrees, 2500.rpm)
            val AT_FAR_TRIANGLE = ShooterState(45.degrees, 0.degrees, 4500.rpm)
        }
    }

}