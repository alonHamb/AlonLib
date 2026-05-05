package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Robot
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.exampleInstantCommand
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem


class RobotContainer(
    hardwareMap: HardwareMap,
    telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad,
    alliance: Alliance,
    telemetryLevel: TelemetryLevel
) : Robot() {

    val controllerA = GamepadEx(gamepad1)
    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem declaration
    val exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry, telemetryLevel)

    // --- init functions ---
    init {
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }

    fun initializeSubsystems() {
        exampleSubsystem

    }

    fun configureButtonBindings() {
        with(controllerA) {
            getGamepadButton(GamepadKeys.Button.X).whenPressed(exampleSubsystem.exampleInstantCommand(0.0))
        }


    }

    fun setDefaultCommands() {
        exampleSubsystem.defaultCommand = exampleSubsystem.exampleInstantCommand { controllerA.leftY }
    }

}
