package org.firstinspires.ftc.teamcode.subsystems.intake

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Intake.INTAKE_MOTOR_ID
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import kotlin.math.absoluteValue

class IntakeSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    // --- hardware declaration ---
    val intakeMotor = HaMotor(hardwareMap, INTAKE_MOTOR_ID, Motor.GoBILDA.RPM_1150).apply {
        zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
    }

    // --- state getters ---

    val isActive get() = intakeMotor.percentOutput.absoluteValue >= 0

    // --- motors control ---

    fun setMotorPower(power: PercentOutput) {
        intakeMotor.percentOutput = power
    }

    fun stopMotor() {
        intakeMotor.stop()
    }

    // --- telemetry ---
    fun addTelemetry() {
        telemetry.addLine("--- intake subsystem ---")
        telemetry.addData("Running Command", super.currentCommand)
        telemetry.addData("is running: ", isActive)
    }

}
