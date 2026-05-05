package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Robot
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance


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


    // --- init functions ---
    init {
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }

    fun initializeSubsystems() {


    }

    fun configureButtonBindings() {
        with(controllerA) {
        }
        with(controllerB) {
            
        }


    }

    fun setDefaultCommands() {
    }

}
