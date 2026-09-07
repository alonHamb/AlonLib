package alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor

class HaOpticalDistanceSensor(val sensor: OpticalDistanceSensor) : HardwareDevice by sensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(OpticalDistanceSensor::class.java, id))

    val lightDetected get() = sensor.lightDetected
    val rawLightDetected get() = sensor.rawLightDetected
    val rawLightDetectedMax get() = sensor.rawLightDetectedMax

    fun enableLed(enable: Boolean) = sensor.enableLed(enable)
}
