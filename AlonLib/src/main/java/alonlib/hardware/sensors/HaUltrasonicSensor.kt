package alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.UltrasonicSensor

class HaUltrasonicSensor(val sensor: UltrasonicSensor) : HardwareDevice by sensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(UltrasonicSensor::class.java, id))

    val ultrasonicLevel get() = sensor.ultrasonicLevel
    val status: String get() = sensor.status()
}
