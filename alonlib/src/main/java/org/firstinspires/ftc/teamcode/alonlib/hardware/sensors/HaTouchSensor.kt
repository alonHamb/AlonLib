package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor

class HaTouchSensor(val touchSensor: TouchSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by touchSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(TouchSensor::class.java, id))

    val value get() = touchSensor.value
    val isPressed get() = touchSensor.isPressed
}
