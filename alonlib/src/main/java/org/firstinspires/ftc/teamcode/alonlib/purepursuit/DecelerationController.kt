package org.firstinspires.ftc.teamcode.alonlib.purepursuit

/** A standalone deceleration profile, independent of [Path]'s own [PathMotionProfile] hook -- for custom deceleration logic used outside a [Path]. */
abstract class DecelerationController {

    private var lastDistanceToTarget = -1.0
    private var lastCallNanos = -1L

    fun process(motorSpeeds: DoubleArray, distanceToTarget: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double) {
        decelerateMotorSpeeds(
            motorSpeeds, distanceToTarget, lastDistanceToTarget, System.nanoTime() - lastCallNanos, configuredMovementSpeed, configuredTurnSpeed,
        )
        lastDistanceToTarget = distanceToTarget
        lastCallNanos = System.nanoTime()
    }

    /**
     * Scales [motorSpeeds] (in place) to slow the robot as it approaches a target. [lastDistanceToTarget]/[timeSinceLastCallNanos]
     * are -1 on the first call, and can otherwise be used to estimate speed.
     */
    abstract fun decelerateMotorSpeeds(
        motorSpeeds: DoubleArray,
        distanceToTarget: Double,
        lastDistanceToTarget: Double,
        timeSinceLastCallNanos: Long,
        configuredMovementSpeed: Double,
        configuredTurnSpeed: Double,
    )
}
