package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem

class TestCommand(var angle: Double) : CommandBase() {


    override fun initialize() {

    }

    override fun execute() {

        TestSubsystem.setServoAngle(angle)
    }
}