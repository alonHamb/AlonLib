package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap

/** A raw analog input pin (e.g. a potentiometer or an absolute-encoder feedback wire) -- see [HaAbsoluteAnalogEncoder] for the normalized-angle version. */
class HaAnalogInput(val hardwareMap: HardwareMap, id: String) : HardwareDevice {

	val sensor: AnalogInput = hardwareMap.get(AnalogInput::class.java, id)

	val voltage get() = sensor.voltage
	val maxVoltage get() = sensor.maxVoltage

	override fun getManufacturer(): HardwareDevice.Manufacturer = sensor.manufacturer

	override fun getDeviceName(): String = sensor.deviceName

	override fun getConnectionInfo(): String? = sensor.connectionInfo

	override fun getVersion(): Int = sensor.version

	override fun resetDeviceConfigurationForOpMode() {
		sensor.resetDeviceConfigurationForOpMode()
	}

	override fun close() {
		sensor.close()
	}
}
