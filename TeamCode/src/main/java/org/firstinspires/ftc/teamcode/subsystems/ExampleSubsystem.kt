package org.firstinspires.ftc.teamcode.subsystems

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rotations
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo

class ExampleSubsystem(hardwareMap: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {
    val testServo = HaServo(hardwareMap, "testServo").apply {
        this.minPosition = (-150.0).degrees
        this.maxPosition = 150.0.degrees
    }

    init {
        super.name = "TestSubsystem"
    }

    fun setServoAngle(angle: Rotation2d) {
        testServo.set(angle.rotations.toDouble())
    }
}
