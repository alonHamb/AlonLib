package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setShooterStateCommand(state: ShooterConstants.ShooterState) {
    this.state = state
}

fun ShooterSubsystem.dynamicShootingCommand(rpm: AngularVelocity, alliance: Alliance): Command =
    run {
        state = ShooterConstants.ShooterState(
            getDynamicHoodAngle(alliance),
            getDynamicHeadingCalc(alliance),
            rpm
        )
    }
