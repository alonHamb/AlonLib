package org.firstinspires.ftc.teamcode.alonlib.math.trajectory

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Transform2d
import kotlin.math.abs
import kotlin.math.pow

/** A time-parameterized sequence of [State]s: pose, curvature, velocity, and acceleration at each point in time. */
class Trajectory(val states: List<State> = emptyList()) {

    val totalTimeSeconds = states.lastOrNull()?.timeSeconds ?: 0.0

    val initialPose get() = sample(0.0).pose

    /** The (possibly interpolated) [State] at [timeSeconds] since the start of the trajectory. */
    fun sample(timeSeconds: Double): State {
        check(states.isNotEmpty()) { "Trajectory cannot be sampled if it has no states." }

        if (timeSeconds <= states[0].timeSeconds) return states[0]
        if (timeSeconds >= totalTimeSeconds) return states.last()

        // Binary search for the first state at or after timeSeconds; low starts at 1 since we need
        // the previous state for interpolation.
        var low = 1
        var high = states.size - 1
        while (low != high) {
            val mid = (low + high) / 2
            if (states[mid].timeSeconds < timeSeconds) low = mid + 1 else high = mid
        }

        val sample = states[low]
        val prevSample = states[low - 1]

        if (abs(sample.timeSeconds - prevSample.timeSeconds) < 1e-9) return sample

        return prevSample.interpolate(sample, (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds))
    }

    /** This trajectory with every pose carried through [transform], relative to the first pose -- e.g. robot-relative to field-relative. */
    fun transformBy(transform: Transform2d): Trajectory {
        val firstPose = states[0].pose
        val newFirstPose = firstPose + transform
        return Trajectory(states.map { it.copy(pose = newFirstPose + (it.pose - firstPose)) })
    }

    /** This trajectory with every pose expressed relative to [pose] instead of the field/origin frame. */
    fun relativeTo(pose: Pose2d) = Trajectory(states.map { it.copy(pose = it.pose.relativeTo(pose)) })

    /** This trajectory followed by [other], whose timestamps are shifted to start where this one ends. */
    fun concatenate(other: Trajectory): Trajectory {
        if (states.isEmpty()) return other
        val combined = states.toMutableList()
        for (i in 1 until other.states.size) {
            combined.add(other.states[i].let { it.copy(timeSeconds = it.timeSeconds + totalTimeSeconds) })
        }
        return Trajectory(combined)
    }

    /**
     * The pose, curvature, velocity, and acceleration at one point in time along a [Trajectory].
     * [accelerationMetersPerSecondSq] is `var` because [TrajectoryParameterizer] back-fills each
     * state's acceleration from the *next* state's velocity delta after the fact.
     */
    data class State(
        val timeSeconds: Double = 0.0,
        val velocityMetersPerSecond: Double = 0.0,
        var accelerationMetersPerSecondSq: Double = 0.0,
        val pose: Pose2d = Pose2d.kZero,
        val curvatureRadPerMeter: Double = 0.0,
    ) {
        /** This state interpolated [i] of the way towards [endValue]. */
        fun interpolate(endValue: State, i: Double): State {
            val newT = lerp(timeSeconds, endValue.timeSeconds, i)
            val deltaT = newT - timeSeconds

            if (deltaT < 0) return endValue.interpolate(this, 1 - i)

            val reversing = velocityMetersPerSecond < 0 || (abs(velocityMetersPerSecond) < 1e-9 && accelerationMetersPerSecondSq < 0)

            val newV = velocityMetersPerSecond + accelerationMetersPerSecondSq * deltaT
            val newS = (velocityMetersPerSecond * deltaT + 0.5 * accelerationMetersPerSecondSq * deltaT.pow(2)) *
                    (if (reversing) -1.0 else 1.0)

            val interpolationFrac = newS / endValue.pose.translation.getDistance(pose.translation)

            return State(
                newT,
                newV,
                accelerationMetersPerSecondSq,
                lerp(pose, endValue.pose, interpolationFrac),
                lerp(curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac),
            )
        }
    }

    companion object {
        private fun lerp(startValue: Double, endValue: Double, t: Double) = startValue + (endValue - startValue) * t
        private fun lerp(startValue: Pose2d, endValue: Pose2d, t: Double) = startValue + (endValue - startValue) * t
    }
}
