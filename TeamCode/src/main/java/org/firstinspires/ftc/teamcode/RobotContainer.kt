package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.exampleInstantCommand
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem


class RobotContainer(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad,
    alliance: Alliance
) {
    // --- Controller decleration ---
    val controllerA = GamepadEx(gamepad1)

    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem decleration
    var exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry)

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
            getGamepadButton(GamepadKeys.Button.X).whenPressed(exampleSubsystem.exampleInstantCommand())
        }


    }

    fun setDefaultCommands() {
        exampleSubsystem.defaultCommand = exampleSubsystem.exampleInstantCommand()
    }

}
