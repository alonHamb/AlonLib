package alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor

class HaTouchSensor(val touchSensor: TouchSensor) : HardwareDevice by touchSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(TouchSensor::class.java, id))

    val value get() = touchSensor.value
    val isPressed get() = touchSensor.isPressed
}
