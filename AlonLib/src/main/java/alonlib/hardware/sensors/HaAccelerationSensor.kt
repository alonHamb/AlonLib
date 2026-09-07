package alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.AccelerationSensor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import alonlib.units.LinearAcceleration
import alonlib.units.gs

class HaAccelerationSensor(val sensor: AccelerationSensor) : HardwareDevice by sensor {

	constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(AccelerationSensor::class.java, id))

	val totalAcceleration: LinearAcceleration get() = (sensor.acceleration.xAccel + sensor.acceleration.yAccel + sensor.acceleration.zAccel).gs

	var xAcceleration: LinearAcceleration = sensor.acceleration.xAccel.gs

	var yAcceleration: LinearAcceleration = sensor.acceleration.yAccel.gs

	var zAcceleration: LinearAcceleration = sensor.acceleration.zAccel.gs

}
