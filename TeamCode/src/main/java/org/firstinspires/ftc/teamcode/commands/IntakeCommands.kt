package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.SHOOTING_POWER_LEVEL
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

fun IntakeSubsystem.defaultIntakeCommand(): Command = run { setMotorPower(0.0) }

fun IntakeSubsystem.intakeCommand(power: PercentOutput): Command =
    run { setMotorPower(power) }

fun IntakeSubsystem.shootCommand(): Command = run { setMotorPower(SHOOTING_POWER_LEVEL) }
