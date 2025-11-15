package org.firstinspires.ftc.teamcode.subsystems.example

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

@Config
object ExampleConstants {

    // --- hardware configs ---

    val EXAMPLE_SERVO_MIN_POSITION = (-150).degrees
    val EXAMPLE_SERVO_MAX_POSITION = 150.degrees

    @JvmField
    val servoPosition = 0.0.degrees
}