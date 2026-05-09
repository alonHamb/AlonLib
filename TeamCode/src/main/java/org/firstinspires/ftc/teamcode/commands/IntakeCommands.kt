package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.INTAKE_POWER_LEVEL
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

fun IntakeSubsystem.defaultIntakeCommand(): Command = run { setMotorPower(0.0) }

fun IntakeSubsystem.IntakeCommand(): Command = run { setMotorPower(INTAKE_POWER_LEVEL) }
