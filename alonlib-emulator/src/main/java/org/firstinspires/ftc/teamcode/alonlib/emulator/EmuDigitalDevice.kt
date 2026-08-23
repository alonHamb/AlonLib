package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannelController
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.TouchSensor
import emulator.hardware.SimDigitalDevice

/** A [TouchSensor] backed by a [SimDigitalDevice]'s boolean [SimDigitalDevice.state]. */
class EmuTouchSensor(private val sim: SimDigitalDevice) : TouchSensor {
    override fun getValue(): Double = if (sim.state) 1.0 else 0.0
    override fun isPressed(): Boolean = sim.state

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuTouchSensor"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}

/**
 * A [DigitalChannel] backed by a [SimDigitalDevice]'s boolean [SimDigitalDevice.state]. [mode]
 * (input/output) is tracked but doesn't change how [sim] can be driven -- there's no real pin
 * electronics underneath to enforce that distinction.
 */
class EmuDigitalChannel(private val sim: SimDigitalDevice) : DigitalChannel {
    private var mode: DigitalChannel.Mode = DigitalChannel.Mode.INPUT

    override fun getMode(): DigitalChannel.Mode = mode
    override fun setMode(mode: DigitalChannel.Mode) {
        this.mode = mode
    }

    @Deprecated("Deprecated in the FTC SDK")
    override fun setMode(mode: DigitalChannelController.Mode) {
        this.mode = mode.migrate()
    }

    override fun getState(): Boolean = sim.state
    override fun setState(state: Boolean) {
        sim.state = state
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuDigitalChannel"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}
