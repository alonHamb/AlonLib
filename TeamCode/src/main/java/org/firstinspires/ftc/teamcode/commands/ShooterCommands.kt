package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setShooterStateCommand(state: ShooterConstants.ShooterState) {
    this.state = state
}

fun ShooterSubsystem.dynamicShootingDefaultCommand(alliance: Alliance): Command =
    run {
        state = ShooterConstants.ShooterState(
            getDynamicHoodAngle(alliance),
            getDynamicHeadingCalc(alliance),
            2000.rpm
        )
    }

fun ShooterSubsystem.dynamicShootingShootCommand(alliance: Alliance): Command =
    run {
        state = ShooterConstants.ShooterState(
            getDynamicHoodAngle(alliance),
            getDynamicHeadingCalc(alliance),
            getDynamicShootingVelocityCalc(alliance)
        )
    }
