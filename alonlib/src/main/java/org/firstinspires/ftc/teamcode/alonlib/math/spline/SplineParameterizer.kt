package org.firstinspires.ftc.teamcode.alonlib.math.spline

import kotlin.math.abs

/**
 * Breaks a [Spline] into a sequence of [PoseWithCurvature] samples, close enough together
 * (recursively bisecting) that a trajectory generator can treat each segment as a constant-radius
 * arc. Ported from FTC-254's spline parameterizer (also what upstream WPILib's is based on).
 */
object SplineParameterizer {

    private const val MAX_DX = 0.127
    private const val MAX_DY = 0.00127
    private const val MAX_DTHETA = 0.0872

    private const val MALFORMED_SPLINE_MESSAGE =
        "Could not parameterize a malformed spline. This means that you probably had two or more " +
                "adjacent waypoints that were very close together with headings in opposing directions."

    private const val MAX_ITERATIONS = 5000

    class MalformedSplineException(message: String) : RuntimeException(message)

    private class StackContents(val t0: Double, val t1: Double)

    fun parameterize(spline: Spline, t0: Double = 0.0, t1: Double = 1.0): List<PoseWithCurvature> {
        val splinePoints = mutableListOf<PoseWithCurvature>()

        splinePoints.add(spline.getPoint(t0) ?: throw MalformedSplineException(MALFORMED_SPLINE_MESSAGE))

        val stack = ArrayDeque<StackContents>()
        stack.addFirst(StackContents(t0, t1))

        var iterations = 0

        while (stack.isNotEmpty()) {
            val current = stack.removeFirst()

            val start = spline.getPoint(current.t0) ?: throw MalformedSplineException(MALFORMED_SPLINE_MESSAGE)
            val end = spline.getPoint(current.t1) ?: throw MalformedSplineException(MALFORMED_SPLINE_MESSAGE)

            val twist = start.pose.log(end.pose)
            if (abs(twist.dy) > MAX_DY || abs(twist.dx) > MAX_DX || abs(twist.dtheta) > MAX_DTHETA) {
                stack.addFirst(StackContents((current.t0 + current.t1) / 2, current.t1))
                stack.addFirst(StackContents(current.t0, (current.t0 + current.t1) / 2))
            } else {
                splinePoints.add(end)
            }

            iterations++
            if (iterations >= MAX_ITERATIONS) {
                throw MalformedSplineException(MALFORMED_SPLINE_MESSAGE)
            }
        }

        return splinePoints
    }
}
