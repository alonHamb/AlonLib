package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance

@Config

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

    // --- init functions ---
    init {
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }

    fun initializeSubsystems() {
    }

    fun configureButtonBindings() {


    }

    fun setDefaultCommands() {

    }

    // --- FTC Dashboard variables ---
    companion object {
        @JvmField
        var input = 0.5
    }

}