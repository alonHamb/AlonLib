package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo

class TestSubsystem(hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {

    val testMotor = HaMotor(hardwareMap, "motor 0", 8192.0, 0.0)
    val testServo = HaServo(hardwareMap, "servo 0", Data.Servos.Mode.FULL_RANGE, Data.Servos.Type.AxonMax)

    fun testAngle(angle: () -> Rotation2d) {
        testServo.position = angle()
        return
    }

    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addData("encoder position", MathUtils.normalizeDegrees(testMotor.position.degrees, true))
            }
        }
    }
}
