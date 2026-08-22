package org.firstinspires.ftc.teamcode.alonlib.hardware

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/** A [SensorDistance] that can also track named target distances and report when they're reached. */
interface SensorDistanceEx : SensorDistance {

    /** A named distance range, in [unit], that [SensorDistanceEx.targetReached] checks a live reading against. */
    class DistanceTarget(
        val unit: DistanceUnit,
        var minThreshold: Double,
        var maxThreshold: Double,
        var name: String = "Distance Target",
    ) {
        init {
            require(minThreshold >= 0) { "Minimum threshold for SensorDistanceEx must be positive" }
            require(maxThreshold >= 0) { "Maximum threshold for SensorDistanceEx must be positive" }
            require(minThreshold <= maxThreshold) { "Minimum threshold for SensorDistanceEx must be less than maximum threshold" }
        }

        /** A target centered on [target], +-5 [unit] either side. */
        constructor(unit: DistanceUnit, target: Double) : this(unit, target - 5, target + 5)

        var target: Double = (minThreshold + maxThreshold) / 2.0
            private set

        fun atTarget(currentDistance: Double) = currentDistance in minThreshold..maxThreshold
    }

    fun targetReached(target: DistanceTarget): Boolean
    fun addTarget(target: DistanceTarget)
    fun addTargets(targets: List<DistanceTarget>)
    fun checkAllTargets(): Map<DistanceTarget, Boolean>
}
