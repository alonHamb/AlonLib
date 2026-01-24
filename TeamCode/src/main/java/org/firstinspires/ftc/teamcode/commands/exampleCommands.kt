package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.commands.andThen
import org.firstinspires.ftc.teamcode.alonlib.robotPrint
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

fun ExampleSubsystem.exampleInstantCommand(exampleValue: Rotation2d) = RunCommand({ setExamplePosition(exampleValue) }, this)


fun ExampleSubsystem.exampleSubsystemCommand(exampleAngle: () -> Rotation2d): Command =
    RunCommand({
        setExamplePosition(10.degrees)
    }) andThen RunCommand({
        setExamplePosition(20.degrees)
    }) andThen RunCommand({
        setExamplePosition(30.degrees)
    })

fun printCommand(message: () -> String): CommandBase = RunCommand({ robotPrint(message()) })