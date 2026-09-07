package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IrSeekerSensor

class HaIrSeekerSensor(val irSeekerSensor: IrSeekerSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by irSeekerSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(IrSeekerSensor::class.java, id))

    var signalDetectedThreshold: Double
        get() = irSeekerSensor.signalDetectedThreshold
        set(value) { irSeekerSensor.signalDetectedThreshold = value }

    var mode: IrSeekerSensor.Mode
        get() = irSeekerSensor.mode
        set(value) { irSeekerSensor.mode = value }

    val signalDetected get() = irSeekerSensor.signalDetected()
    val angle get() = irSeekerSensor.angle
    val strength get() = irSeekerSensor.strength
    val individualSensors: Array<IrSeekerSensor.IrSeekerIndividualSensor> get() = irSeekerSensor.individualSensors
}
