package org.firstinspires.ftc.teamcode.Commands


import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem

fun TestSubsystem.testDefaultCommand(): Command {
    return run { testServo.percentOutput = 1.0 }
}

fun TestSubsystem.testCommand(angle: () -> Rotation2d): Command {
    return runOnce { testAngle { angle() } }
}
