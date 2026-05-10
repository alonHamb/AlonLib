package org.firstinspires.ftc.teamcode.Commands


import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem

fun TestSubsystem.testDefaultCommand(): Command {
    return run { test() }
}
