package org.firstinspires.ftc.teamcode.commands

import com.hamosad1657.lib.units.degrees
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.commands.andThen
import org.firstinspires.ftc.teamcode.alonlib.commands.instantCommand
import org.firstinspires.ftc.teamcode.alonlib.robotPrint
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

fun ExampleSubsystem.exampleInstantCommand(testAngle: () -> Rotation2d) = RunCommand({ this.setServoAngle(testAngle()) }, this)

fun ExampleSubsystem.exampleSubsystemCommand(exampleAngle: () -> Rotation2d): Command =
    instantCommand {
        setServoAngle(5.degrees)
    } andThen instantCommand {
        setServoAngle(exampleAngle())
    } andThen instantCommand {
        setServoAngle(exampleAngle())
    }

fun printCommand(message: () -> String): CommandBase = RunCommand({ robotPrint(message()) })
