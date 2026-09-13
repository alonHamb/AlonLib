package alonlib.hardware.sensors

import alonlib.hardware.HardwareDevice
import alonlib.units.Angle
import alonlib.units.degrees
import alonlib.units.radians
import alonlib.units.rotations
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

/** A stub for an absolute analog encoder (e.g. an Axon servo's feedback wire) read through an [AnalogInput], normalized to `[0, max)`. */
open class HaAbsoluteAnalogEncoder(
	private val encoder: AnalogInput,
	private val id: String = "",
	range: Angle = 1.rotations,
) : HardwareDevice {

	constructor(hardwareMap: HardwareMap, id: String, range: Angle = 3.3.radians) :
			this(hardwareMap.get(AnalogInput::class.java, id), id, range)

	val angle = ((encoder.voltage / encoder.maxVoltage) * range.asDegrees).degrees

	override fun disable() {
		encoder.close()
	}

	override fun getDeviceType() = "Absolute Analog Encoder; $id"

}
