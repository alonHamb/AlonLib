package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem

fun exampleInstantCommand(exampleSubsystem: ExampleSubsystem, testAngle: () -> Double) =
    RunCommand({ exampleSubsystem.setServoAngle(testAngle()) }, exampleSubsystem)

fun printCommand(value: Double) = InstantCommand({ print(value) })