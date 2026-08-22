package org.firstinspires.ftc.teamcode.alonlib.math.spline

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import kotlin.math.hypot
import kotlin.math.pow

/** A 2D parametric spline (a curve from `t=0` to `t=1`) -- see [CubicHermiteSpline]/[QuinticHermiteSpline]. */
abstract class Spline(private val degree: Int) {

    /** Row 0 = x coefficients, row 1 = y, rows 2-3 = their 1st derivatives, rows 4-5 = 2nd derivatives. */
    abstract fun coefficients(): Matrix

    abstract fun initialControlVector(): ControlVector
    abstract fun finalControlVector(): ControlVector

    /** The pose and curvature at [t] (0 = start, 1 = end), or null where the spline's velocity is ~zero (heading undefined). */
    fun getPoint(t: Double): PoseWithCurvature? {
        val polynomialBases = Matrix(degree + 1, 1)
        val coefficients = coefficients()

        for (i in 0..degree) {
            polynomialBases[i, 0] = t.pow(degree - i)
        }

        val combined = coefficients * polynomialBases

        val x = combined[0, 0]
        val y = combined[1, 0]

        val dx: Double
        val dy: Double
        val ddx: Double
        val ddy: Double

        if (t == 0.0) {
            dx = coefficients[2, degree - 1]
            dy = coefficients[3, degree - 1]
            ddx = coefficients[4, degree - 2]
            ddy = coefficients[5, degree - 2]
        } else {
            dx = combined[2, 0] / t
            dy = combined[3, 0] / t
            ddx = combined[4, 0] / t / t
            ddy = combined[5, 0] / t / t
        }

        if (hypot(dx, dy) < 1e-6) return null

        val curvature = (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * hypot(dx, dy))

        return PoseWithCurvature(Pose2d(x, y, Rotation2d(dx, dy)), curvature)
    }

    /**
     * A control point for a spline: [x]/[y] are each `[position, velocity, ...]` -- the value of
     * each successive derivative at that end of the spline.
     */
    class ControlVector(x: DoubleArray, y: DoubleArray) {
        val x = x.copyOf()
        val y = y.copyOf()
    }
}
