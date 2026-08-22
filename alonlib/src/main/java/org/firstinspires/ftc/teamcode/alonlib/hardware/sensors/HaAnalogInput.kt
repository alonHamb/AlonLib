package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

/** A raw analog input pin (e.g. a potentiometer or an absolute-encoder feedback wire) -- see [HaAbsoluteAnalogEncoder] for the normalized-angle version. */
class HaAnalogInput(val analogInput: AnalogInput) : com.qualcomm.robotcore.hardware.HardwareDevice by analogInput {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(AnalogInput::class.java, id))

    val voltage get() = analogInput.voltage
    val maxVoltage get() = analogInput.maxVoltage
}
