package alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor

/** Typically the Control/Expansion Hub's own battery voltage sensor (`hardwareMap.voltageSensor` also enumerates every one on the robot). */
class HaVoltageSensor(val voltageSensor: VoltageSensor) : HardwareDevice by voltageSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(VoltageSensor::class.java, id))

    val voltage get() = voltageSensor.voltage
}
