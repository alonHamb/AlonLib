package org.firstinspires.ftc.teamcode.alonlib.hardware

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/** The REV 2m time-of-flight distance sensor, with named [SensorDistanceEx.DistanceTarget] tracking. */
open class SensorRevTOFDistance(
    private val distanceSensor: DistanceSensor,
    targets: List<SensorDistanceEx.DistanceTarget> = emptyList(),
) : SensorDistanceEx {

    constructor(hardwareMap: HardwareMap, name: String, targets: List<SensorDistanceEx.DistanceTarget> = emptyList()) :
            this(hardwareMap.get(DistanceSensor::class.java, name), targets)

    private val targetList = targets.toMutableList()

    override fun getDistance(unit: DistanceUnit): Double = distanceSensor.getDistance(unit)

    override fun targetReached(target: SensorDistanceEx.DistanceTarget) = target.atTarget(getDistance(target.unit))

    override fun addTarget(target: SensorDistanceEx.DistanceTarget) {
        if (target !in targetList) targetList.add(target)
    }

    override fun addTargets(targets: List<SensorDistanceEx.DistanceTarget>) {
        targets.forEach { addTarget(it) }
    }

    override fun checkAllTargets(): Map<SensorDistanceEx.DistanceTarget, Boolean> =
        targetList.associateWith { it.atTarget(getDistance(it.unit)) }

    override fun disable() = distanceSensor.close()
    override fun getDeviceType() = "TOF Rev 2m Distance Sensor"
}
