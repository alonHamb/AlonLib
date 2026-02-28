package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setShooterStateCommand(state: ShooterConstants.ShooterState) {
    this.state = state
}

fun ShooterSubsystem.dynamicShootingCalcCommand(alliance: Alliance) {
    state = ShooterConstants.ShooterState(getDynamicHoodAngle(alliance), getDynamicHeadingCalc(alliance), 0.rpm)
}

fun ShooterSubsystem.shootCommand(alliance: Alliance) {
    currentVelocitySetPoint = getDynamicShootingVelocityCalc(alliance)
    if (isInVelocityTolerance && ) {

    }
}

