package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Intake.INTAKE_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.INTAKE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.TRANSFER_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.TRANSFER_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import kotlin.math.absoluteValue

@Config
class TransferSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    @JvmField
    // --- hardware declaration
    val intakeMotor = HaMotor(hardwareMap, INTAKE_MOTOR_ID, INTAKE_MOTOR_TYPE).apply {
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        setRunMode(Motor.RunMode.RawPower)
        runningDirection = Motor.Direction.FORWARD
    }

    val transferMotor = HaMotor(hardwareMap, TRANSFER_MOTOR_ID, TRANSFER_MOTOR_TYPE).apply {
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        setRunMode(Motor.RunMode.RawPower)
        runningDirection = Motor.Direction.REVERSE
    }

    // --- state getters ---

    val isRunning get() = transferMotor.percentOutput.absoluteValue > 0

    // --- motor control ---

    fun setMotorPower(power: PercentOutput) {
        transferMotor.percentOutput = power
    }

    fun stopMotor() {
        transferMotor.disable()
    }

    // --- telemetry ---
    fun addTelemetry() {
        telemetry.addLine("--- transfer subsystem ---")
        telemetry.addData("Running Command", super.currentCommand)
        telemetry.addData("is running", isRunning)
        telemetry.addData("running direction", transferMotor.runningDirection.toString())
    }


}
