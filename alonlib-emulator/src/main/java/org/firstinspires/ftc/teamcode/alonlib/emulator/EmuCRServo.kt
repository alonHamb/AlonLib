package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.ServoController
import emulator.hardware.SimServo
import emulator.hardware.Direction as SimDirection

/**
 * A [CRServo] backed by a [SimServo]. The emulator's config format doesn't distinguish a
 * continuous-rotation servo from a positional one -- a `<Servo>`/`<CRServo>` tag both resolve to
 * the same [SimServo] (see [EmuServoController]) -- so which SDK interface an OpMode asks
 * `hardwareMap.get` for is what decides how it's driven, and both adapters get registered under
 * the same device name (see [buildEmulatedHardwareMap]).
 *
 * Unlike [EmuDcMotorEx]/[emulatedServo], a real CR servo has no position feedback -- [getPower]
 * only ever reports back the last commanded value -- so this doesn't drive [SimServo]'s
 * positional slew dynamics at all; it just tracks the commanded power itself.
 */
class EmuCRServo(private val sim: SimServo) : CRServo {
    private var powerField = 0.0

    override fun setDirection(direction: DcMotorSimple.Direction) {
        sim.direction = if (direction == DcMotorSimple.Direction.REVERSE) SimDirection.REVERSE else SimDirection.FORWARD
    }

    override fun getDirection(): DcMotorSimple.Direction =
        if (sim.direction == SimDirection.REVERSE) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

    override fun setPower(power: Double) {
        powerField = power.coerceIn(-1.0, 1.0)
    }

    override fun getPower(): Double = powerField

    override fun getController(): ServoController = EmuCRServoController()
    override fun getPortNumber(): Int = sim.port.index

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuCRServo"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {
        powerField = 0.0
    }

    override fun close() {}
}

/**
 * A minimal [ServoController] so [EmuCRServo.getController] doesn't crash code that calls through
 * it -- real subsystem code almost never does, so this doesn't need to be wired back to [SimServo].
 */
private class EmuCRServoController : ServoController {
    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuCRServoController"
    override fun getConnectionInfo(): String = "emulated"
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}

    override fun pwmEnable() {}
    override fun pwmDisable() {}
    override fun getPwmStatus(): ServoController.PwmStatus = ServoController.PwmStatus.ENABLED
    override fun setServoPosition(servo: Int, position: Double) {}
    override fun getServoPosition(servo: Int): Double = 0.0
}
