package org.firstinspires.ftc.teamcode

import com.hamosad1657.lib.units.degrees
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.exampleInstantCommand
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

class RobotContainer(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad
) {
    // --- Controller decleration ---
    val controllerA = GamepadEx(gamepad1)

    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem decleration
    lateinit var exampleSubsystem: ExampleSubsystem

    init {
        telemetry.addLine("Robot Container initializing")
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
        telemetry.addLine("Robot Container initialized")
    }


    fun initializeSubsystems() {
        exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry)
    }

    fun configureButtonBindings() {
        with(controllerA) {
            getGamepadButton(GamepadKeys.Button.A).whenPressed(
                exampleSubsystem.exampleInstantCommand { 150.degrees }
            )
        }

    }

    fun setDefaultCommands() {
        exampleSubsystem.defaultCommand = exampleSubsystem.exampleInstantCommand { 0.degrees }
    }

}