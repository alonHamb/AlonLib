package org.firstinspires.ftc.teamcode.alonlib.math.control

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * A trapezoid-shaped velocity profile: accelerate at [Constraints.maxAcceleration] up to
 * [Constraints.maxVelocity], cruise, then decelerate into the goal.
 *
 * Typical usage: keep a [State] across loop iterations, and each loop call
 * `state = profile.calculate(dt, state, goalState)` -- the unprofiled `goalState` is free to
 * change between calls, and `calculate` returns it unchanged whenever it's already reachable
 * within the constraints.
 */
class TrapezoidProfile(private val constraints: Constraints) {

    private var direction = 1
    private lateinit var current: State

    private var endAccel = 0.0
    private var endFullSpeed = 0.0
    private var endDecel = 0.0

    data class Constraints(val maxVelocity: Double, val maxAcceleration: Double)

    data class State(val position: Double = 0.0, val velocity: Double = 0.0)

    /** The position/velocity at time [t] after [current], profiled towards [goal]. */
    fun calculate(t: Double, current: State, goal: State): State {
        direction = if (shouldFlipAcceleration(current, goal)) -1 else 1
        this.current = direct(current)
        val goal = direct(goal)

        var currentVelocity = this.current.velocity
        if (currentVelocity > constraints.maxVelocity) {
            currentVelocity = constraints.maxVelocity
        }
        val currentPosition = this.current.position
        this.current = State(currentPosition, currentVelocity)

        // Deal with a possibly truncated profile (nonzero initial/final velocity) by computing
        // parameters as if it began/ended at zero velocity, then trimming.
        val cutoffBegin = currentVelocity / constraints.maxAcceleration
        val cutoffDistBegin = cutoffBegin * cutoffBegin * constraints.maxAcceleration / 2.0

        val cutoffEnd = goal.velocity / constraints.maxAcceleration
        val cutoffDistEnd = cutoffEnd * cutoffEnd * constraints.maxAcceleration / 2.0

        val fullTrapezoidDist = cutoffDistBegin + (goal.position - currentPosition) + cutoffDistEnd
        var accelerationTime = constraints.maxVelocity / constraints.maxAcceleration

        var fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * constraints.maxAcceleration

        // The profile never reaches full speed.
        if (fullSpeedDist < 0) {
            accelerationTime = sqrt(fullTrapezoidDist / constraints.maxAcceleration)
            fullSpeedDist = 0.0
        }

        endAccel = accelerationTime - cutoffBegin
        endFullSpeed = endAccel + fullSpeedDist / constraints.maxVelocity
        endDecel = endFullSpeed + accelerationTime - cutoffEnd

        var resultPosition = currentPosition
        var resultVelocity = currentVelocity

        val result = when {
            t < endAccel      -> {
                resultVelocity += t * constraints.maxAcceleration
                resultPosition += (currentVelocity + t * constraints.maxAcceleration / 2.0) * t
                State(resultPosition, resultVelocity)
            }

            t < endFullSpeed   -> {
                resultVelocity = constraints.maxVelocity
                resultPosition += (currentVelocity + endAccel * constraints.maxAcceleration / 2.0) * endAccel +
                        constraints.maxVelocity * (t - endAccel)
                State(resultPosition, resultVelocity)
            }

            t <= endDecel      -> {
                resultVelocity = goal.velocity + (endDecel - t) * constraints.maxAcceleration
                val timeLeft = endDecel - t
                resultPosition = goal.position - (goal.velocity + timeLeft * constraints.maxAcceleration / 2.0) * timeLeft
                State(resultPosition, resultVelocity)
            }

            else               -> goal
        }

        return direct(result)
    }

    /** How much longer, from the state passed to the most recent [calculate], until [target] is reached. */
    fun timeLeftUntil(target: Double): Double {
        var position = current.position * direction
        var velocity = current.velocity * direction

        var endAccelLocal = endAccel * direction
        var endFullSpeedLocal = endFullSpeed * direction - endAccelLocal

        if (target < position) {
            endAccelLocal = -endAccelLocal
            endFullSpeedLocal = -endFullSpeedLocal
            velocity = -velocity
        }

        endAccelLocal = max(endAccelLocal, 0.0)
        endFullSpeedLocal = max(endFullSpeedLocal, 0.0)

        val acceleration = constraints.maxAcceleration
        val deceleration = -constraints.maxAcceleration

        val distToTarget = abs(target - position)
        if (distToTarget < 1e-6) return 0.0

        var accelDist = velocity * endAccelLocal + 0.5 * acceleration * endAccelLocal * endAccelLocal

        val decelVelocity = if (endAccelLocal > 0) {
            sqrt(abs(velocity * velocity + 2 * acceleration * accelDist))
        } else {
            velocity
        }

        var fullSpeedDist = constraints.maxVelocity * endFullSpeedLocal
        val decelDist: Double

        when {
            accelDist > distToTarget                    -> {
                accelDist = distToTarget
                fullSpeedDist = 0.0
                decelDist = 0.0
            }

            accelDist + fullSpeedDist > distToTarget -> {
                fullSpeedDist = distToTarget - accelDist
                decelDist = 0.0
            }

            else                                          -> {
                decelDist = distToTarget - fullSpeedDist - accelDist
            }
        }

        val accelTime = (-velocity + sqrt(abs(velocity * velocity + 2 * acceleration * accelDist))) / acceleration
        val decelTime = (-decelVelocity + sqrt(abs(decelVelocity * decelVelocity + 2 * deceleration * decelDist))) / deceleration
        val fullSpeedTime = fullSpeedDist / constraints.maxVelocity

        return accelTime + fullSpeedTime + decelTime
    }

    /** The total time the profile takes, from the state passed to the most recent [calculate], to reach the goal. */
    fun totalTime() = endDecel

    /** @returns true once [t] (seconds since the profile started) has passed [totalTime]. */
    fun isFinished(t: Double) = t >= totalTime()

    private fun shouldFlipAcceleration(initial: State, goal: State) = initial.position > goal.position

    private fun direct(state: State) = State(state.position * direction, state.velocity * direction)
}
