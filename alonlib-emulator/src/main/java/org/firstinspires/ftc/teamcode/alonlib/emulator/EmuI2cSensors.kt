package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.CompassSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import emulator.hardware.SimI2cDevice
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * A [NormalizedColorSensor]/[ColorSensor]/[DistanceSensor] all in one, backed by a single
 * [SimI2cDevice] -- matching a real REV Color Sensor V3, which
 * [org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaColorSensor] force-casts to both
 * of the latter two. [SimI2cDevice] has no color/distance physics of its own (see its doc
 * comment) -- drive it from your test/adapter code with [SimI2cDevice.setReading]: `"red"`/
 * `"green"`/`"blue"`/`"alpha"` (normalized, `[0,1)`, matching [NormalizedRGBA]'s own range) and
 * `"distanceMm"`.
 */
class EmuColorSensor(private val sim: SimI2cDevice) : NormalizedColorSensor, ColorSensor, DistanceSensor {
    private var gainField = 1f
    private var address = I2cAddr.zero()

    override fun getNormalizedColors(): NormalizedRGBA = NormalizedRGBA().apply {
        red = sim.getReading("red").toFloat()
        green = sim.getReading("green").toFloat()
        blue = sim.getReading("blue").toFloat()
        alpha = sim.getReading("alpha").toFloat()
    }

    override fun getGain(): Float = gainField
    override fun setGain(newGain: Float) {
        gainField = newGain
    }

    override fun red(): Int = (sim.getReading("red") * 255).toInt().coerceIn(0, 255)
    override fun green(): Int = (sim.getReading("green") * 255).toInt().coerceIn(0, 255)
    override fun blue(): Int = (sim.getReading("blue") * 255).toInt().coerceIn(0, 255)
    override fun alpha(): Int = (sim.getReading("alpha") * 255).toInt().coerceIn(0, 255)
    override fun argb(): Int = (alpha() shl 24) or (red() shl 16) or (green() shl 8) or blue()

    override fun enableLed(enable: Boolean) {}
    override fun setI2cAddress(newAddress: I2cAddr) {
        address = newAddress
    }

    override fun getI2cAddress(): I2cAddr = address

    override fun getDistance(unit: DistanceUnit): Double = unit.fromUnit(DistanceUnit.MM, sim.getReading("distanceMm"))

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuColorSensor"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}

/** A [CompassSensor] backed by one [SimI2cDevice]'s `"headingDeg"` reading, driven via [SimI2cDevice.setReading]. */
class EmuCompassSensor(private val sim: SimI2cDevice) : CompassSensor {
    private var mode = CompassSensor.CompassMode.MEASUREMENT_MODE

    override fun getDirection(): Double = ((sim.getReading("headingDeg") % 360.0) + 360.0) % 360.0
    override fun status(): String = "emulated"
    override fun setMode(mode: CompassSensor.CompassMode) {
        this.mode = mode
    }

    override fun calibrationFailed(): Boolean = false

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuCompassSensor"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}
