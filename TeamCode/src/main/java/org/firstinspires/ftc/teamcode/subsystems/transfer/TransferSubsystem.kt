package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.LEFT_TRANSFER_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.RIGHT_TRANSFER_SERVO_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferConstants.TRANSFER_GEAR_RATIO
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferConstants.TRANSFER_SERVO_MODE
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferConstants.TRANSFER_SERVO_TYPE
import kotlin.math.absoluteValue

@Config
class TransferSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {
    @JvmField
    // --- hardware declaration
    val leftTransferServo = HaServo(hardwareMap, LEFT_TRANSFER_SERVO_ID, TRANSFER_SERVO_MODE, TRANSFER_SERVO_TYPE).apply {
        runningDirection = Motor.Direction.FORWARD
    }
    val rightTransferServo = HaServo(hardwareMap, RIGHT_TRANSFER_SERVO_ID, TRANSFER_SERVO_MODE, TRANSFER_SERVO_TYPE).apply {
        runningDirection = Motor.Direction.REVERSE
    }

    // --- state getters ---
    val isRunning get() = power.absoluteValue > 0

    var velocity: AngularVelocity = 0.rpm
        set(value) {
            leftTransferServo.velocity = (value.asRpm / TRANSFER_GEAR_RATIO).rpm
            rightTransferServo.velocity = (value.asRpm / TRANSFER_GEAR_RATIO).rpm
            field = value
        }

    var power: PercentOutput = 0.0
        set(value) {
            leftTransferServo.percentOutput = value
            rightTransferServo.percentOutput = value
            field = value
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
                telemetry.addData("running direction", "${leftTransferServo.runningDirection}")
                telemetry.addData("velocity", velocity.asRpm)
                telemetry.addData("power level", power)
            }
        }
    }

    // --- periodic function ---
    override fun periodic() {
        updateTelemetry()
    }
}
