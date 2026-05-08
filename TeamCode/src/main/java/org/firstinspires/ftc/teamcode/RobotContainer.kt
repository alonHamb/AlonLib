package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Robot
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
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
    alliance: Alliance,
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
