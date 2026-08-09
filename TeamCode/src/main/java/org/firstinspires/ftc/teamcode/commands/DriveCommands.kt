package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

fun DriveSubsystem.driveFieldCentricCommand(
    xSpeed: () -> Double,
    ySpeed: () -> Double,
    turnSpeed: () -> Double
                                           ): Command =
    run { fieldCentricDrive(xSpeed(), ySpeed(), turnSpeed()) }

fun DriveSubsystem.resetImuCommand(): Command = run { resetImu() }
