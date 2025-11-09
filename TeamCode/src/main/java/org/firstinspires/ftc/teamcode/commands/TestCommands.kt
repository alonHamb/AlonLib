package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem

fun testCommand(testSubsystem: TestSubsystem, testAngle: () -> Double) =
    RunCommand({ testSubsystem.setServoAngle(testAngle()) }, testSubsystem)

fun printCommand(value: Double) = InstantCommand({ print(value) })