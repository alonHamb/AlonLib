package org.firstinspires.ftc.teamcode.subsystems.example

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_CR_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_SERVO_RANGE
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaCrServo
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_MOTOR_CPR
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_MOTOR_KD
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_MOTOR_KI
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_MOTOR_KP
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_MOTOR_RPM

class ExampleSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry, var telemetryLevel: TelemetryLevel) : SubsystemBase() {
    // --- hardware initialization and configuration ---

    //here you declare all the hardware used by your subsystem and configure them.
    //each hardware device has its own value

    val motor = HaMotor(hardwareMap, EXAMPLE_MOTOR_ID, EXAMPLE_MOTOR_CPR, EXAMPLE_MOTOR_RPM).apply {
        runMode = Motor.RunMode.RawPower
        pidfGains = PIDGains(EXAMPLE_MOTOR_KP, EXAMPLE_MOTOR_KI, EXAMPLE_MOTOR_KD, 0.0)
        runningDirection = Motor.Direction.REVERSE
        zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
    }
    val encoder = HaMotor(hardwareMap, "motor 0", 8192, 312)
    val servo = HaServo(hardwareMap, EXAMPLE_SERVO_ID, EXAMPLE_SERVO_RANGE)

    val crServo = HaCrServo(hardwareMap, EXAMPLE_CR_SERVO_ID)

    // --- state getters ---

    //here you create a properties for any state that you want eiter to receive in other areas of the code or to let other areas of the code set and modify
    val stateComponent1 get() = servo.position

    val stateComponent2 get() = motor.velocity


    //  --- subsystemFunctions --

    //here you can put any function your subsystem might need but beware! most things can be getters and setters and don't require a function
    fun setExamplePosition() {
        servo.position = encoder.position
    }

    // --- Telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Testing -> {
                telemetry.addData("servo position", servo.position.degrees)
                telemetry.addData("encoder position", encoder.position.degrees)
            }

            TelemetryLevel.Competition -> {}
        }
    }

    override fun periodic() {
        //here you set all the actions you want to run every run loop
        motor.update()
        updateTelemetry()

    }

}
