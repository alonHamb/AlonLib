package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.driveFieldCentricCommand
import org.firstinspires.ftc.teamcode.commands.dynamicShootingCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem


class RobotContainer(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad,
    val alliance: Alliance
) {
    // --- Controller decleration ---
    val controllerA = GamepadEx(gamepad1)

    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem decleration
    var shooterSubsystem = ShooterSubsystem(hardwareMap, telemetry)
    var driveSubsystem = DriveSubsystem(hardwareMap, telemetry)

    // --- init functions ---
    init {
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }

    fun initializeSubsystems() {
        shooterSubsystem
        driveSubsystem


    }

    fun configureButtonBindings() {
        with(controllerA) {

        }


    }

    fun setDefaultCommands() {
        driveSubsystem.defaultCommand = driveSubsystem.driveFieldCentricCommand(controllerA.leftX, controllerA.leftY, controllerA.rightX)
        shooterSubsystem.defaultCommand = RunCommand({ shooterSubsystem.dynamicShootingCommand(alliance) })
    }

}