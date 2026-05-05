package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

fun ExampleSubsystem.exampleInstantCommand(position: Double) =
    RunCommand({ setExamplePosition(position) }, this)

fun ExampleSubsystem.exampleInstantCommand(position: () -> Double): Command =
    RunCommand({ setExamplePosition(position()) }, this)

fun ExampleSubsystem.exampleSubsystemCommand(exampleAngle: () -> Rotation2d): Command =
    RunCommand({ setExamplePosition(exampleAngle().degrees) }, this)
