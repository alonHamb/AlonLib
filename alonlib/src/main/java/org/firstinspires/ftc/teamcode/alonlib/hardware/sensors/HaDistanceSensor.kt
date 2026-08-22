package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class HaDistanceSensor(val distanceSensor: DistanceSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by distanceSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(DistanceSensor::class.java, id))

    fun getDistance(unit: DistanceUnit): Double = distanceSensor.getDistance(unit)
}
