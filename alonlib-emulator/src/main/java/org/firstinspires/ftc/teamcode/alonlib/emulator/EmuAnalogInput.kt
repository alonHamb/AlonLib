package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.AnalogInputController
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor
import com.qualcomm.robotcore.util.SerialNumber
import emulator.hardware.SimAnalogDevice

/** Backs one [SimAnalogDevice] as an [AnalogInputController] -- the [AnalogInput] equivalent of [EmuServoController]. */
private class EmuAnalogInputController(private val sim: SimAnalogDevice) : AnalogInputController {
    override fun getAnalogInputVoltage(channel: Int): Double = sim.voltage
    override fun getMaxAnalogInputVoltage(): Double = 3.3
    override fun getSerialNumber(): SerialNumber = SerialNumber.createFake()

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuAnalogInputController"
    override fun getConnectionInfo(): String = "emulated"
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}

/**
 * Builds a genuine [AnalogInput] backed by [sim]. [AnalogInput] is a concrete class, not an
 * interface like most of the SDK's hardware types -- similar to [emulatedServo] with `ServoImplEx`,
 * this only overrides [AnalogInput.getDeviceName], since the real implementation reaches
 * `AppUtil.getDefContext()`, a real Android resource lookup unavailable on a desktop JVM.
 */
fun emulatedAnalogInput(sim: SimAnalogDevice): AnalogInput = object : AnalogInput(EmuAnalogInputController(sim), 0) {
    override fun getDeviceName(): String = "EmuAnalogInput"
}

/** An [OpticalDistanceSensor] backed by a [SimAnalogDevice]'s raw [SimAnalogDevice.voltage], scaled by its 0-3.3V default range. */
class EmuOpticalDistanceSensor(private val sim: SimAnalogDevice) : OpticalDistanceSensor {
    override fun getLightDetected(): Double = (sim.voltage / 3.3).coerceIn(0.0, 1.0)
    override fun getRawLightDetected(): Double = sim.voltage
    override fun getRawLightDetectedMax(): Double = 3.3
    override fun enableLed(enable: Boolean) {}
    override fun status(): String = "emulated"

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuOpticalDistanceSensor"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}
