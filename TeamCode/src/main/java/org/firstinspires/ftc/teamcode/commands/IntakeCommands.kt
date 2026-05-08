package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

fun IntakeSubsystem.defaultIntakeCommand(): Command = run { setMotorPower(0.0) }
