package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.math.PIDGains
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedRotations

@Config
class TestSubsystem(hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {

    val encoder = HaMotor(hardwareMap, "m0", 8192.0, 0.0)

    val motor = HaMotor(hardwareMap, "m3", Motor.GoBILDA.RPM_312).apply {
        runMode = Motor.RunMode.RawPower
        pidfGains = PIDGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    }

    fun test() {
        motor.voltage = 12.0
    }

    override fun periodic() {
        motor.update()
        updateTelemetry()
    }

    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing     -> {
                telemetry.addData("encoder position", encoder.position.normalizedRotations)
                telemetry.addData("motor position", motor.position.normalizedDegrees)
                telemetry.addData("motor velocity", motor.velocity.asRpm)
                telemetry.addData("motor error", motor.error)
                telemetry.addData("motor setpoint", motor.setPoint)
                telemetry.addData("motor power", motor.percentOutput)
                telemetry.addData("motor gains", motor.pidfGains)
                telemetry.addData("motor voltage", motor.voltage)
            }
        }
    }
}
