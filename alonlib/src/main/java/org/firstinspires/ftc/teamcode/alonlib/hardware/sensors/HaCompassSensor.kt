package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.CompassSensor
import com.qualcomm.robotcore.hardware.HardwareMap

class HaCompassSensor(val sensor: CompassSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by sensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(CompassSensor::class.java, id))

    val direction get() = sensor.direction
    val calibrationFailed get() = sensor.calibrationFailed()

    fun setMode(mode: CompassSensor.CompassMode) = sensor.setMode(mode)
}
