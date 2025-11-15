package org.firstinspires.ftc.teamcode.subsystems.example

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_SERVO_ID
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_SERVO_MAX_POSITION
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_SERVO_MIN_POSITION

class ExampleSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    // --- hardware initialization and configuration ---
    val exampleServo: HaServo = HaServo(
        hardwareMap,
        EXAMPLE_SERVO_ID,
        EXAMPLE_SERVO_MIN_POSITION,
        EXAMPLE_SERVO_MAX_POSITION
    ).apply {
        // state
    }

    fun setServoAngle(angle: Rotation2d) {
        exampleServo.setPositionSetPoint(angle)
    }

}