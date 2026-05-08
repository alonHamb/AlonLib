package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferSubsystem

fun TransferSubsystem.defaultTransferCommand(): Command = run { power = 0.0 }
