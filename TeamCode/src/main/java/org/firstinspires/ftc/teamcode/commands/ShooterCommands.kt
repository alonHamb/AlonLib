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

fun ShooterSubsystem.ShootCommand(alliance: Alliance): Command =
    run {
        when (this.shootingMode) {
            ShooterConstants.ShootingModes.DYNAMIC -> {
                state.velocity = getDynamicShootingVelocityCalc(alliance)
            }

            ShooterConstants.ShootingModes.SETPOINTS -> {}
        }
        state.velocity = getDynamicShootingVelocityCalc(alliance)
    }
