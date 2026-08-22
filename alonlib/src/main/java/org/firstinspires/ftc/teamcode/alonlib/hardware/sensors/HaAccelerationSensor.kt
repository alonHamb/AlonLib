package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.AccelerationSensor
import com.qualcomm.robotcore.hardware.HardwareMap

class HaAccelerationSensor(val sensor: AccelerationSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by sensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(AccelerationSensor::class.java, id))

    val acceleration get() = sensor.acceleration
}
