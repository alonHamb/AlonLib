package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations

class TestSubsystem(hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {

    val testMotor = HaMotor(hardwareMap, "m0", 8192.0, 0.0)
    val testServo = HaServo(hardwareMap, "s0", Data.Servos.Mode.FULL_RANGE, Data.Servos.Type.AxonMax).apply {
        runningDirection = Motor.Direction.REVERSE

    }

    var controllerPosition: Rotation2d = 0.0.rotations

    fun testAngle(angle: () -> Rotation2d) {
        testServo.position = angle()
        controllerPosition = angle()
    }

    override fun periodic() {
        updateTelemetry()
    }

    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addData("controller position", controllerPosition.normalizedDegrees)
                telemetry.addData("encoder position", MathUtils.normalizeDegrees(testMotor.position.degrees, true))
            }
        }
    }
}
