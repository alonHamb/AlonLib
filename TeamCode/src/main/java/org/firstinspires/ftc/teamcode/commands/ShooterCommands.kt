package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.COASTING_VELOCITY
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setShooterStateCommand(state: ShooterConstants.ShooterState) {
    this.state = state
}

fun ShooterSubsystem.ShootingDefaultCommand(alliance: Alliance): Command =
    run {
        state = when (shootingMode) {
            ShooterConstants.ShootingModes.SETPOINTS -> ShooterConstants.ShooterState.DISABLED_STATE
            ShooterConstants.ShootingModes.DYNAMIC -> ShooterConstants.ShooterState(
                getDynamicHoodAngle(alliance),
                getDynamicHeadingCalc(alliance),
                COASTING_VELOCITY
            )
        }


    }

fun ShooterSubsystem.changeModeCommand(): Command = run {
    shootingMode = when (shootingMode) {
        ShooterConstants.ShootingModes.DYNAMIC -> ShooterConstants.ShootingModes.SETPOINTS
        ShooterConstants.ShootingModes.SETPOINTS -> ShooterConstants.ShootingModes.DYNAMIC
    }
}

fun ShooterSubsystem.shootCommand(alliance: Alliance): Command =
    run {
        when (this.shootingMode) {
            ShooterConstants.ShootingModes.DYNAMIC -> {
                state.velocity = getDynamicShootingVelocityCalc(alliance)
            }

            ShooterConstants.ShootingModes.SETPOINTS -> {}
        }
        state.velocity = getDynamicShootingVelocityCalc(alliance)
    }
