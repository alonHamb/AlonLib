package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.CompassSensor
import com.qualcomm.robotcore.hardware.CompassSensor.CompassMode

import com.qualcomm.robotcore.hardware.HardwareMap

class HaCompassSensor(val sensor: CompassSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by sensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(CompassSensor::class.java, id))

    val direction get() = sensor.direction
    val calibrationFailed get() = sensor.calibrationFailed()

    var mode: CompassMode = CompassMode.MEASUREMENT_MODE
        set(value) {
            field = value
            sensor.setMode(value)
        }

    fun setMode(mode: CompassMode) = sensor.setMode(mode)
}
