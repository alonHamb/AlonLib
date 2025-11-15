package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.exampleInstantCommand
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.servoPosition
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
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }


    fun initializeSubsystems() {
        exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry)
    }

    fun configureButtonBindings() {
        with(controllerA) {
            getGamepadButton(GamepadKeys.Button.A).whenPressed(
                exampleSubsystem.exampleInstantCommand { servoPosition }
            )
        }

    }

    fun setDefaultCommands() {
        //exampleSubsystem.defaultCommand =
        // exampleSubsystem.exampleInstantCommand { (((controllerA.leftY + 1) / 2) * 300).degrees }
    }

}