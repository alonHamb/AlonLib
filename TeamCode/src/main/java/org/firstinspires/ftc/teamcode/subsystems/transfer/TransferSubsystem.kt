package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Transfer.TRANSFER_MOTOR_ID
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import kotlin.math.absoluteValue

@Config
class TransferSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    @JvmField
    // --- hardware decleration
    val transferMotor = HaMotor(hardwareMap, TRANSFER_MOTOR_ID, Motor.GoBILDA.BARE).apply {
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        setRunMode(Motor.RunMode.RawPower)
    }

    // --- state getters ---

    val isRunning get() = transferMotor.precentOutput.absoluteValue > 0

    // --- motor control ---

    fun setMotorPower(power: PercentOutput) {
        transferMotor.precentOutput = power
    }

    fun stopMotor() {
        transferMotor.disable()
    }

    // --- telemetry ---
    fun addTelemetry() {
        telemetry.addData("is running", isRunning)
        telemetry.addData("running direction", transferMotor.runningDirection.toString())
    }


}