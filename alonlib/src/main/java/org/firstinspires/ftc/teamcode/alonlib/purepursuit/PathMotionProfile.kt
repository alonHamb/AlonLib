package org.firstinspires.ftc.teamcode.alonlib.purepursuit

/** Shapes how a [Path] speeds up leaving a waypoint and slows down approaching one -- implement [accelerate]/[decelerate] for a custom profile. */
abstract class PathMotionProfile {

    private var lastDistance = -1.0
    private var lastCallNanos = -1L
    private var lastCallWasDecelerate = true

    /** Called by [Path] every loop while approaching a target; relays to [decelerate] with the robot's estimated speed. */
    fun processDecelerate(motorSpeeds: DoubleArray, distanceToTarget: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double) {
        if (lastCallWasDecelerate) {
            val speed = (lastDistance - distanceToTarget) / ((System.nanoTime() - lastCallNanos) * 1e9)
            decelerate(motorSpeeds, distanceToTarget, speed, configuredMovementSpeed, configuredTurnSpeed)
        } else {
            lastCallWasDecelerate = true
        }
        lastDistance = distanceToTarget
        lastCallNanos = System.nanoTime()
    }

    /** Called by [Path] every loop while leaving a waypoint; relays to [accelerate] with the robot's estimated speed. */
    fun processAccelerate(motorSpeeds: DoubleArray, distanceFromTarget: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double) {
        if (!lastCallWasDecelerate) {
            val speed = (distanceFromTarget - lastDistance) / ((System.nanoTime() - lastCallNanos) * 1e9)
            accelerate(motorSpeeds, distanceFromTarget, speed, configuredMovementSpeed, configuredTurnSpeed)
        } else {
            lastCallWasDecelerate = false
        }
        lastDistance = distanceFromTarget
        lastCallNanos = System.nanoTime()
    }

    /** Scales [motorSpeeds] (in place) to slow the robot as it approaches a target, [distanceToTarget] away and moving at [speed] units/sec. */
    abstract fun decelerate(motorSpeeds: DoubleArray, distanceToTarget: Double, speed: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double)

    /** Scales [motorSpeeds] (in place) to speed the robot up leaving a target, [distanceFromTarget] away and moving at [speed] units/sec. */
    abstract fun accelerate(motorSpeeds: DoubleArray, distanceFromTarget: Double, speed: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double)
}
