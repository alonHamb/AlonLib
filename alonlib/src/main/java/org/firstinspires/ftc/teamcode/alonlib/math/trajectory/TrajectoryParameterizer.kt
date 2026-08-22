package org.firstinspires.ftc.teamcode.alonlib.math.trajectory

import org.firstinspires.ftc.teamcode.alonlib.math.spline.PoseWithCurvature
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint.TrajectoryConstraint
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Time-parameterizes a sequence of spline [PoseWithCurvature] points into a [Trajectory] by
 * running a forward pass (accelerate as much as allowed) and a backward pass (decelerate as much
 * as required) over the [TrajectoryConstraint]s, then integrating the resulting velocity profile.
 * See http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf for the derivation.
 */
object TrajectoryParameterizer {

    class TrajectoryGenerationException(message: String) : RuntimeException(message)

    private class ConstrainedState(
        var pose: PoseWithCurvature,
        var distanceMeters: Double,
        var maxVelocityMetersPerSecond: Double,
        var minAccelerationMetersPerSecondSq: Double,
        var maxAccelerationMetersPerSecondSq: Double,
    )

    fun timeParameterizeTrajectory(
        points: List<PoseWithCurvature>,
        constraints: List<TrajectoryConstraint>,
        startVelocityMetersPerSecond: Double,
        endVelocityMetersPerSecond: Double,
        maxVelocityMetersPerSecond: Double,
        maxAccelerationMetersPerSecondSq: Double,
        reversed: Boolean,
    ): Trajectory {
        val constrainedStates = ArrayList<ConstrainedState>(points.size)
        var predecessor = ConstrainedState(
            points[0], 0.0, startVelocityMetersPerSecond, -maxAccelerationMetersPerSecondSq, maxAccelerationMetersPerSecondSq,
        )

        // Forward pass: at each point, go as fast as global limits, custom constraints, and the
        // previous point's reachable acceleration allow.
        for (i in points.indices) {
            val constrainedState = ConstrainedState(points[i], 0.0, 0.0, 0.0, 0.0)
            constrainedStates.add(constrainedState)

            val ds = constrainedState.pose.pose.translation.getDistance(predecessor.pose.pose.translation)
            constrainedState.distanceMeters = predecessor.distanceMeters + ds

            // Iterate since acceleration limits may themselves depend on velocity.
            while (true) {
                constrainedState.maxVelocityMetersPerSecond = min(
                    maxVelocityMetersPerSecond,
                    sqrt(
                        predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond +
                                predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0,
                    ),
                )

                constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq
                constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq

                for (constraint in constraints) {
                    constrainedState.maxVelocityMetersPerSecond = min(
                        constrainedState.maxVelocityMetersPerSecond,
                        constraint.getMaxVelocityMetersPerSecond(
                            constrainedState.pose.pose,
                            constrainedState.pose.curvatureRadPerMeter,
                            constrainedState.maxVelocityMetersPerSecond,
                        ),
                    )
                }

                enforceAccelerationLimits(reversed, constraints, constrainedState)

                if (ds < 1e-6) break

                val actualAcceleration = (
                        constrainedState.maxVelocityMetersPerSecond * constrainedState.maxVelocityMetersPerSecond -
                                predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond
                        ) / (ds * 2.0)

                if (constrainedState.maxAccelerationMetersPerSecondSq < actualAcceleration - 1e-6) {
                    predecessor.maxAccelerationMetersPerSecondSq = constrainedState.maxAccelerationMetersPerSecondSq
                } else {
                    if (actualAcceleration > predecessor.minAccelerationMetersPerSecondSq) {
                        predecessor.maxAccelerationMetersPerSecondSq = actualAcceleration
                    }
                    // If still too low, the backward pass will fix it up.
                    break
                }
            }
            predecessor = constrainedState
        }

        var successor = ConstrainedState(
            points.last(),
            constrainedStates.last().distanceMeters,
            endVelocityMetersPerSecond,
            -maxAccelerationMetersPerSecondSq,
            maxAccelerationMetersPerSecondSq,
        )

        // Backward pass: cap velocities so the trajectory can still decelerate in time.
        for (i in points.indices.reversed()) {
            val constrainedState = constrainedStates[i]
            val ds = constrainedState.distanceMeters - successor.distanceMeters // negative

            while (true) {
                val newMaxVelocity = sqrt(
                    successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond +
                            successor.minAccelerationMetersPerSecondSq * ds * 2.0,
                )

                if (newMaxVelocity >= constrainedState.maxVelocityMetersPerSecond) break

                constrainedState.maxVelocityMetersPerSecond = newMaxVelocity

                enforceAccelerationLimits(reversed, constraints, constrainedState)

                if (ds > -1e-6) break

                val actualAcceleration = (
                        constrainedState.maxVelocityMetersPerSecond * constrainedState.maxVelocityMetersPerSecond -
                                successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond
                        ) / (ds * 2.0)

                if (constrainedState.minAccelerationMetersPerSecondSq > actualAcceleration + 1e-6) {
                    successor.minAccelerationMetersPerSecondSq = constrainedState.minAccelerationMetersPerSecondSq
                } else {
                    successor.minAccelerationMetersPerSecondSq = actualAcceleration
                    break
                }
            }
            successor = constrainedState
        }

        // Integrate the constrained states forward in time into trajectory states.
        val states = ArrayList<Trajectory.State>(points.size)
        var timeSeconds = 0.0
        var distanceMeters = 0.0
        var velocityMetersPerSecond = 0.0

        for (i in constrainedStates.indices) {
            val state = constrainedStates[i]

            val ds = state.distanceMeters - distanceMeters
            val accel = (
                    state.maxVelocityMetersPerSecond * state.maxVelocityMetersPerSecond -
                            velocityMetersPerSecond * velocityMetersPerSecond
                    ) / (ds * 2)

            var dt = 0.0
            if (i > 0) {
                states[i - 1].accelerationMetersPerSecondSq = if (reversed) -accel else accel
                dt = when {
                    abs(accel) > 1e-6 -> (state.maxVelocityMetersPerSecond - velocityMetersPerSecond) / accel
                    abs(velocityMetersPerSecond) > 1e-6 -> ds / velocityMetersPerSecond
                    else -> throw TrajectoryGenerationException("Something went wrong at iteration $i of time parameterization.")
                }
            }

            velocityMetersPerSecond = state.maxVelocityMetersPerSecond
            distanceMeters = state.distanceMeters
            timeSeconds += dt

            states.add(
                Trajectory.State(
                    timeSeconds,
                    if (reversed) -velocityMetersPerSecond else velocityMetersPerSecond,
                    if (reversed) -accel else accel,
                    state.pose.pose,
                    state.pose.curvatureRadPerMeter,
                ),
            )
        }

        return Trajectory(states)
    }

    private fun enforceAccelerationLimits(reverse: Boolean, constraints: List<TrajectoryConstraint>, state: ConstrainedState) {
        for (constraint in constraints) {
            val factor = if (reverse) -1.0 else 1.0
            val minMaxAccel = constraint.getMinMaxAccelerationMetersPerSecondSq(
                state.pose.pose,
                state.pose.curvatureRadPerMeter,
                state.maxVelocityMetersPerSecond * factor,
            )

            if (minMaxAccel.minAccelerationMetersPerSecondSq > minMaxAccel.maxAccelerationMetersPerSecondSq) {
                throw TrajectoryGenerationException("Infeasible trajectory constraint: ${constraint::class.java.name}\n")
            }

            state.minAccelerationMetersPerSecondSq = max(
                state.minAccelerationMetersPerSecondSq,
                if (reverse) -minMaxAccel.maxAccelerationMetersPerSecondSq else minMaxAccel.minAccelerationMetersPerSecondSq,
            )

            state.maxAccelerationMetersPerSecondSq = min(
                state.maxAccelerationMetersPerSecondSq,
                if (reverse) -minMaxAccel.minAccelerationMetersPerSecondSq else minMaxAccel.maxAccelerationMetersPerSecondSq,
            )
        }
    }
}
