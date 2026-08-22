package org.firstinspires.ftc.teamcode.alonlib.math.trajectory

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Transform2d
import org.firstinspires.ftc.teamcode.alonlib.math.spline.PoseWithCurvature
import org.firstinspires.ftc.teamcode.alonlib.math.spline.Spline
import org.firstinspires.ftc.teamcode.alonlib.math.spline.SplineHelper
import org.firstinspires.ftc.teamcode.alonlib.math.spline.SplineParameterizer

/** Builds [Trajectory]s from waypoints/control vectors via clamped-cubic or quintic-hermite splines. */
object TrajectoryGenerator {

    private val kFlip = Transform2d(Translation2d.kZero, Rotation2d.kPi)
    private val kDoNothingTrajectory = Trajectory(listOf(Trajectory.State()))

    /** Clamped cubic splines through [initial]/[interiorWaypoints]/[end]'s exterior control vectors. */
    fun generateTrajectory(
        initial: Spline.ControlVector,
        interiorWaypoints: List<Translation2d>,
        end: Spline.ControlVector,
        config: TrajectoryConfig,
    ): Trajectory {
        val newInitial = Spline.ControlVector(initial.x, initial.y)
        val newEnd = Spline.ControlVector(end.x, end.y)

        if (config.reversed) {
            newInitial.x[1] *= -1
            newInitial.y[1] *= -1
            newEnd.x[1] *= -1
            newEnd.y[1] *= -1
        }

        val points = try {
            splinePointsFromSplines(
                SplineHelper.getCubicSplinesFromControlVectors(newInitial, interiorWaypoints.toTypedArray(), newEnd).toList(),
            )
        } catch (ex: SplineParameterizer.MalformedSplineException) {
            robotPrintError(ex.message)
            return kDoNothingTrajectory
        }

        return parameterize(if (config.reversed) unflip(points) else points, config)
    }

    /** Clamped cubic splines through [start]/[interiorWaypoints]/[end], choosing interior headings for continuous curvature. */
    fun generateTrajectory(start: Pose2d, interiorWaypoints: List<Translation2d>, end: Pose2d, config: TrajectoryConfig): Trajectory {
        val controlVectors = SplineHelper.getCubicControlVectorsFromWaypoints(start, interiorWaypoints.toTypedArray(), end)
        return generateTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config)
    }

    /** Quintic hermite splines through fully-specified [controlVectors] (guarantees continuous curvature). */
    fun generateTrajectoryFromControlVectors(controlVectors: List<Spline.ControlVector>, config: TrajectoryConfig): Trajectory {
        val newControlVectors = controlVectors.map { vector ->
            val newVector = Spline.ControlVector(vector.x, vector.y)
            if (config.reversed) {
                newVector.x[1] *= -1
                newVector.y[1] *= -1
            }
            newVector
        }

        val points = try {
            splinePointsFromSplines(SplineHelper.getQuinticSplinesFromControlVectors(newControlVectors.toTypedArray()).toList())
        } catch (ex: SplineParameterizer.MalformedSplineException) {
            robotPrintError(ex.message)
            return kDoNothingTrajectory
        }

        return parameterize(if (config.reversed) unflip(points) else points, config)
    }

    /** Quintic hermite splines through [waypoints] (guarantees continuous curvature). */
    fun generateTrajectory(waypoints: List<Pose2d>, config: TrajectoryConfig): Trajectory {
        val newWaypoints = if (config.reversed) waypoints.map { it + kFlip } else waypoints

        val points = try {
            splinePointsFromSplines(
                SplineHelper.optimizeCurvature(SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints)).toList(),
            )
        } catch (ex: SplineParameterizer.MalformedSplineException) {
            robotPrintError(ex.message)
            return kDoNothingTrajectory
        }

        return parameterize(if (config.reversed) unflip(points) else points, config)
    }

    /** Parameterizes [splines] by arc length into the [PoseWithCurvature] samples a trajectory is time-parameterized from. */
    fun splinePointsFromSplines(splines: List<Spline>): List<PoseWithCurvature> {
        val splinePoints = mutableListOf<PoseWithCurvature>()

        splinePoints.add(splines[0].getPoint(0.0) ?: throw SplineParameterizer.MalformedSplineException(MALFORMED_SPLINE_MESSAGE))

        for (spline in splines) {
            val points = SplineParameterizer.parameterize(spline)
            // Drop the first point of each spline -- it duplicates the previous spline's last point.
            splinePoints.addAll(points.subList(1, points.size))
        }
        return splinePoints
    }

    private fun unflip(points: List<PoseWithCurvature>) =
        points.map { PoseWithCurvature(it.pose + kFlip, -it.curvatureRadPerMeter) }

    private fun parameterize(points: List<PoseWithCurvature>, config: TrajectoryConfig) = TrajectoryParameterizer.timeParameterizeTrajectory(
        points,
        config.constraints,
        config.startVelocityMetersPerSecond,
        config.endVelocityMetersPerSecond,
        config.maxVelocityMetersPerSecond,
        config.maxAccelerationMetersPerSecondSq,
        config.reversed,
    )

    private const val MALFORMED_SPLINE_MESSAGE =
        "Could not parameterize a malformed spline. This means that you probably had two or more " +
                "adjacent waypoints that were very close together with headings in opposing directions."
}
