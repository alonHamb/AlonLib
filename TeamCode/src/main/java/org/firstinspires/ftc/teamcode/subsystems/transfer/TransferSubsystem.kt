package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.LEFT_TRANSFER_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.RIGHT_TRANSFER_SERVO_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.motors.HaCrServo
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import kotlin.math.absoluteValue

@Config
class TransferSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {
    @JvmField
    // --- hardware declaration
    val leftTransferServo = HaCrServo(hardwareMap, LEFT_TRANSFER_SERVO_ID).apply {
        zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
        runningDirection = Motor.Direction.FORWARD
    }
    val rightTransferServo = HaCrServo(hardwareMap, RIGHT_TRANSFER_SERVO_ID).apply {
        zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
        runningDirection = Motor.Direction.REVERSE
    }

    // --- state getters ---

    val isRunning get() = leftTransferServo.percentOutput.absoluteValue > 0

    // --- motor control ---

    fun setMotorPower(power: PercentOutput) {
        leftTransferServo.percentOutput = power
        rightTransferServo.percentOutput = power
    }


    fun stopMotor() {
        leftTransferServo.stop()
        rightTransferServo.stop()
    }

    // --- telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addLine("--- transfer subsystem ---")
                telemetry.addData("Running Command", super.currentCommand)
                telemetry.addData("is running", isRunning)
                telemetry.addData("running direction", leftTransferServo.runningDirection.toString())
            }
        }
    }

    // --- periodic function ---
    override fun periodic() {
        updateTelemetry()
    }
}
