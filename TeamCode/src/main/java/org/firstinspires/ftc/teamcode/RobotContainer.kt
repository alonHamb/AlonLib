package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.robot.Robot
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem

class RobotContainer(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad
) : Robot() {
    init {
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }

    val controllerA = GamepadEx(gamepad1)

    val controllerB = GamepadEx(gamepad2)

    fun initializeSubsystems() {
        this.ExampleSubsystem = ExampleSubsystem(hardwareMap, telemetry)
    }

    fun configureButtonBindings() {

    }

    fun setDefaultCommands() {


    }

}