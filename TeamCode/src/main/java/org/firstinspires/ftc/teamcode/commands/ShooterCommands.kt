package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setShooterStateCommand(state: ShooterConstants.ShooterState) {
    this.state = state
}

fun ShooterSubsystem.dynamicShootingCommand(alliance: Alliance) {
    if (this.limelight.isWithinLimelightAccuracyRange && this.limelight.isWithinLimelightAccuracyRange)
        this.dynamicShootingUpdate(alliance)
    else if ()
}

