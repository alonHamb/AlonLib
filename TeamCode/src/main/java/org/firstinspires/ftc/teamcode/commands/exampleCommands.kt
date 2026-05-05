package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.robotPrint
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

fun ExampleSubsystem.exampleInstantCommand(position: Double) =
    RunCommand({ setExamplePosition(position) }, this)

fun ExampleSubsystem.exampleInstantCommand(position: () -> Double) =
    RunCommand({ setExamplePosition(position()) }, this)

fun ExampleSubsystem.exampleSubsystemCommand(exampleAngle: () -> Rotation2d): Command =
    RunCommand({ setExamplePosition(exampleAngle().degrees) }, this)

fun printCommand(message: () -> String) = RunCommand({ robotPrint(message()) })
