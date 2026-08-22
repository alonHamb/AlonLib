package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/** Wraps a [DistanceSensor] (works with any vendor, e.g. the REV 2m time-of-flight sensor), with optional named [DistanceTarget] tracking. */
class HaDistanceSensor(
    val distanceSensor: DistanceSensor,
    targets: List<DistanceTarget> = emptyList(),
) : com.qualcomm.robotcore.hardware.HardwareDevice by distanceSensor {

    constructor(hardwareMap: HardwareMap, id: String, targets: List<DistanceTarget> = emptyList()) :
            this(hardwareMap.get(DistanceSensor::class.java, id), targets)

    /** A named distance range, in [unit], that [targetReached] checks a live reading against. */
    class DistanceTarget(
        val unit: DistanceUnit,
        var minThreshold: Double,
        var maxThreshold: Double,
        var name: String = "Distance Target",
    ) {
        init {
            require(minThreshold >= 0) { "Minimum threshold for DistanceTarget must be positive" }
            require(maxThreshold >= 0) { "Maximum threshold for DistanceTarget must be positive" }
            require(minThreshold <= maxThreshold) { "Minimum threshold for DistanceTarget must be less than maximum threshold" }
        }

        /** A target centered on [target], +-5 [unit] either side. */
        constructor(unit: DistanceUnit, target: Double) : this(unit, target - 5, target + 5)

        var target: Double = (minThreshold + maxThreshold) / 2.0
            private set

        fun atTarget(currentDistance: Double) = currentDistance in minThreshold..maxThreshold
    }

    private val targetList = targets.toMutableList()

    fun getDistance(unit: DistanceUnit): Double = distanceSensor.getDistance(unit)

    fun targetReached(target: DistanceTarget) = target.atTarget(getDistance(target.unit))

    fun addTarget(target: DistanceTarget) {
        if (target !in targetList) targetList.add(target)
    }

    fun addTargets(targets: List<DistanceTarget>) {
        targets.forEach { addTarget(it) }
    }

    fun checkAllTargets(): Map<DistanceTarget, Boolean> =
        targetList.associateWith { it.atTarget(getDistance(it.unit)) }
}
