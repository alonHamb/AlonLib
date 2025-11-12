package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem

fun exampleInstantCommand(exampleSubsystem: ExampleSubsystem, testAngle: () -> Rotation2d) =
    RunCommand({ exampleSubsystem.setServoAngle(testAngle()) }, exampleSubsystem)


fun printCommand(value: Double) = InstantCommand({ print(value) })