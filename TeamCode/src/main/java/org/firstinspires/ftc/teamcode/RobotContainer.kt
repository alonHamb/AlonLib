package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Robot
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.IntakeCommand
import org.firstinspires.ftc.teamcode.commands.defaultIntakeCommand
import org.firstinspires.ftc.teamcode.commands.defaultTransferCommand
import org.firstinspires.ftc.teamcode.commands.driveFieldCentricCommand
import org.firstinspires.ftc.teamcode.commands.dynamicShootingDefaultCommand
import org.firstinspires.ftc.teamcode.commands.resetImuCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferSubsystem


/**
 * this is the robot container class it contains all of your robots subsystems and the controllers you are going to use
 */
class RobotContainer(
    hardwareMap: HardwareMap,
    telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad,
    val alliance: Alliance,
    telemetryLevel: TelemetryLevel
                    ) : Robot() {
    val controllerA = GamepadEx(gamepad1)
    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem declaration
    val driveSubsystem = DriveSubsystem(hardwareMap, telemetry, telemetryLevel)
    val intakeSubsystem = IntakeSubsystem(hardwareMap, telemetry, telemetryLevel)
    val transferSubsystem = TransferSubsystem(hardwareMap, telemetry, telemetryLevel)
    val shooterSubsystem = ShooterSubsystem(hardwareMap, telemetry, telemetryLevel)

    // --- init functions ---
    init {
        configureButtonBindings()
        setDefaultCommands()
    }

    fun configureButtonBindings() {
        with(controllerA) {
            GamepadButton(this, GamepadKeys.Button.A).whileHeld(intakeSubsystem.IntakeCommand())
            GamepadButton(this, GamepadKeys.Button.OPTIONS).whenPressed(driveSubsystem.resetImuCommand())
        
        }
        with(controllerB) {

        }


    }

    fun setDefaultCommands() {
        driveSubsystem.defaultCommand = driveSubsystem.driveFieldCentricCommand({ controllerA.leftX }, { controllerA.leftY }) { controllerA.rightX }
        intakeSubsystem.defaultCommand = intakeSubsystem.defaultIntakeCommand()
        transferSubsystem.defaultCommand = transferSubsystem.defaultTransferCommand()
        shooterSubsystem.defaultCommand = shooterSubsystem.dynamicShootingDefaultCommand(alliance)
    }

}
