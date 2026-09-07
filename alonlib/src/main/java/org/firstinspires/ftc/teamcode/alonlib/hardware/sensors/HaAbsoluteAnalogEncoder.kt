package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.alonlib.hardware.HardwareDevice
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.units.radians
import org.firstinspires.ftc.teamcode.alonlib.units.rotations

/** A stub for an absolute analog encoder (e.g. an Axon servo's feedback wire) read through an [AnalogInput], normalized to `[0, max)` of [angleUnit]. */
open class HaAbsoluteAnalogEncoder(
	private val encoder: AnalogInput,
	private val id: String = "",
	private val range: Rotation2d = 1.rotations,
) : HardwareDevice {

	constructor(hardwareMap: HardwareMap, id: String, range: Rotation2d = 3.3.radians) :
			this(hardwareMap.get(AnalogInput::class.java, id), id, range)

	override fun disable() {
		// No-op, matching upstream -- AnalogInput.close() isn't called here.
	}

	override fun getDeviceType() = "Absolute Analog Encoder; $id"

}
