package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

fun DriveSubsystem.driveFieldCentricCommand(xSpeed: Double, ySpeed: Double, turnSpeed: Double): Command =
    RunCommand({ fieldCentricDrive(xSpeed, ySpeed, turnSpeed) })
