package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import emulator.hardware.SimMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import emulator.hardware.Direction as SimDirection
import emulator.hardware.RunMode as SimRunMode
import emulator.hardware.ZeroPowerBehavior as SimZeroPowerBehavior

/**
 * A [DcMotorEx] backed by a [SimMotor] from ftc-control-hub-emulator, so real code that talks to
 * `DcMotor`/`DcMotorEx` -- directly, or via
 * [org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor], which wraps one -- runs
 * completely unmodified against simulated dynamics instead of a real motor controller.
 *
 * PID/current-alert configuration is accepted but not modeled: [HaMotor] runs its own PIDF loop in
 * software and writes the result as [setPower]/[setVelocity], so the motor-controller-side PIDF
 * knobs this class stubs out are never exercised by AlonLib's own hardware layer.
 */
class EmuDcMotorEx(val sim: SimMotor) : DcMotorEx {

    private var motorTypeField: MotorConfigurationType = MotorConfigurationType().apply {
        setTicksPerRev(sim.ticksPerRev)
        setMaxRPM(sim.maxRpm)
    }
    private var motorEnabled = true
    private var currentAlertAmps = 0.0

    // --- DcMotorSimple ---

    override fun setDirection(direction: DcMotorSimple.Direction) {
        sim.direction = if (direction == DcMotorSimple.Direction.REVERSE) SimDirection.REVERSE else SimDirection.FORWARD
    }

    override fun getDirection(): DcMotorSimple.Direction =
        if (sim.direction == SimDirection.REVERSE) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

    override fun setPower(power: Double) = sim.setPower(power)
    override fun getPower(): Double = sim.getPower()

    // --- HardwareDevice ---

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuDcMotorEx"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() = sim.resetEncoder()
    override fun close() {}

    // --- DcMotor ---

    override fun getMotorType(): MotorConfigurationType = motorTypeField
    override fun setMotorType(motorType: MotorConfigurationType) {
        motorTypeField = motorType
    }

    override fun getController(): DcMotorController = EmuDcMotorController(sim)
    override fun getPortNumber(): Int = sim.port.index

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        sim.zeroPowerBehavior = if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT) SimZeroPowerBehavior.FLOAT else SimZeroPowerBehavior.BRAKE
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior =
        if (sim.zeroPowerBehavior == SimZeroPowerBehavior.FLOAT) DcMotor.ZeroPowerBehavior.FLOAT else DcMotor.ZeroPowerBehavior.BRAKE

    @Deprecated("Deprecated in the FTC SDK")
    override fun setPowerFloat() {
        sim.zeroPowerBehavior = SimZeroPowerBehavior.FLOAT
        sim.setPower(0.0)
    }

    override fun getPowerFloat(): Boolean = sim.getPower() == 0.0 && sim.zeroPowerBehavior == SimZeroPowerBehavior.FLOAT

    override fun setTargetPosition(position: Int) {
        sim.targetPosition = position
    }

    override fun getTargetPosition(): Int = sim.targetPosition

    override fun isBusy(): Boolean =
        sim.mode == SimRunMode.RUN_TO_POSITION && sim.getCurrentPosition() != sim.targetPosition

    override fun getCurrentPosition(): Int = sim.getCurrentPosition()

    override fun setMode(mode: DcMotor.RunMode) {
        sim.mode = when (mode.migrate()) {
            DcMotor.RunMode.RUN_USING_ENCODER -> SimRunMode.RUN_USING_ENCODER
            DcMotor.RunMode.RUN_TO_POSITION -> SimRunMode.RUN_TO_POSITION
            DcMotor.RunMode.STOP_AND_RESET_ENCODER -> SimRunMode.STOP_AND_RESET_ENCODER
            else -> SimRunMode.RUN_WITHOUT_ENCODER
        }
    }

    override fun getMode(): DcMotor.RunMode = when (sim.mode) {
        SimRunMode.RUN_USING_ENCODER -> DcMotor.RunMode.RUN_USING_ENCODER
        SimRunMode.RUN_TO_POSITION -> DcMotor.RunMode.RUN_TO_POSITION
        SimRunMode.STOP_AND_RESET_ENCODER -> DcMotor.RunMode.STOP_AND_RESET_ENCODER
        SimRunMode.RUN_WITHOUT_ENCODER -> DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    // --- DcMotorEx ---

    override fun setMotorEnable() {
        motorEnabled = true
    }

    override fun setMotorDisable() {
        motorEnabled = false
        sim.setPower(0.0)
    }

    override fun isMotorEnabled(): Boolean = motorEnabled

    override fun setVelocity(angularRate: Double) {
        val maxTicksPerSec = (sim.maxRpm / 60.0) * sim.ticksPerRev
        sim.setPower((angularRate / maxTicksPerSec).coerceIn(-1.0, 1.0))
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit) {
        val ticksPerSec = AngleUnit.RADIANS.fromUnit(unit, angularRate) * sim.ticksPerRev / (2 * Math.PI)
        setVelocity(ticksPerSec)
    }

    override fun getVelocity(): Double = sim.getVelocity()

    override fun getVelocity(unit: AngleUnit): Double =
        unit.fromUnit(AngleUnit.RADIANS, sim.getVelocity() * 2 * Math.PI / sim.ticksPerRev)

    @Deprecated("Deprecated in the FTC SDK")
    override fun setPIDCoefficients(mode: DcMotor.RunMode, pidCoefficients: PIDCoefficients) {}

    override fun setPIDFCoefficients(mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) {}
    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {}
    override fun setPositionPIDFCoefficients(p: Double) {}

    @Deprecated("Deprecated in the FTC SDK")
    override fun getPIDCoefficients(mode: DcMotor.RunMode): PIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)

    override fun getPIDFCoefficients(mode: DcMotor.RunMode): PIDFCoefficients = PIDFCoefficients(0.0, 0.0, 0.0, 0.0)

    override fun setTargetPositionTolerance(tolerance: Int) {}
    override fun getTargetPositionTolerance(): Int = 0

    override fun getCurrent(unit: CurrentUnit): Double = unit.convert(sim.currentDrawAmps(), CurrentUnit.AMPS)
    override fun getCurrentAlert(unit: CurrentUnit): Double = unit.convert(currentAlertAmps, CurrentUnit.AMPS)
    override fun setCurrentAlert(current: Double, unit: CurrentUnit) {
        currentAlertAmps = unit.toAmps(current)
    }

    override fun isOverCurrent(): Boolean = currentAlertAmps > 0.0 && sim.currentDrawAmps() > currentAlertAmps
}

/**
 * Minimal [DcMotorController] returned by [EmuDcMotorEx.getController]. Real subsystem code
 * almost never calls through the controller instead of the motor directly; this exists only so
 * that code which does (logging a connection string, etc.) doesn't crash.
 */
private class EmuDcMotorController(private val sim: SimMotor) : DcMotorController {
    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuDcMotorController"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}

    override fun setMotorType(motor: Int, motorType: MotorConfigurationType) {}
    override fun getMotorType(motor: Int): MotorConfigurationType = MotorConfigurationType.getUnspecifiedMotorType()
    override fun setMotorMode(motor: Int, mode: DcMotor.RunMode) {}
    override fun getMotorMode(motor: Int): DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    override fun setMotorPower(motor: Int, power: Double) = sim.setPower(power)
    override fun getMotorPower(motor: Int): Double = sim.getPower()
    override fun isBusy(motor: Int): Boolean = false
    override fun setMotorZeroPowerBehavior(motor: Int, zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {}
    override fun getMotorZeroPowerBehavior(motor: Int): DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    override fun getMotorPowerFloat(motor: Int): Boolean = false
    override fun setMotorTargetPosition(motor: Int, position: Int) {}
    override fun getMotorTargetPosition(motor: Int): Int = 0
    override fun getMotorCurrentPosition(motor: Int): Int = sim.getCurrentPosition()
    override fun resetDeviceConfigurationForOpMode(motor: Int) {}
}
