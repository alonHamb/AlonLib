package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo

class TestSubsystem(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
) : SubsystemBase() {
    val testServo = HaServo(hardwareMap, "testServo")

    init {
        super.name = "TestSubsystem"
    }

    fun setServoAngle(angle: Double) {
        testServo.set(angle)
    }
}
