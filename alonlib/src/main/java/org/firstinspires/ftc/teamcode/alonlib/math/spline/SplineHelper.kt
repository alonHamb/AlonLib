package org.firstinspires.ftc.teamcode.alonlib.math.spline

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import kotlin.math.hypot

/** Builds [CubicHermiteSpline]s/[QuinticHermiteSpline]s from waypoints or control vectors. */
object SplineHelper {

    /** The 2 cubic control vectors (endpoints only) for a path through [start], [interiorWaypoints], [end]. */
    fun getCubicControlVectorsFromWaypoints(
        start: Pose2d,
        interiorWaypoints: Array<Translation2d>,
        end: Pose2d,
    ): Array<Spline.ControlVector> {
        val initialCV: Spline.ControlVector
        val endCV: Spline.ControlVector

        // Chooses a magnitude automatically that makes the splines look better.
        if (interiorWaypoints.isEmpty()) {
            val scalar = start.translation.getDistance(end.translation) * 1.2
            initialCV = cubicControlVector(scalar, start)
            endCV = cubicControlVector(scalar, end)
        } else {
            val startScalar = start.translation.getDistance(interiorWaypoints[0]) * 1.2
            initialCV = cubicControlVector(startScalar, start)
            val endScalar = end.translation.getDistance(interiorWaypoints.last()) * 1.2
            endCV = cubicControlVector(endScalar, end)
        }
        return arrayOf(initialCV, endCV)
    }

    fun getQuinticSplinesFromWaypoints(waypoints: List<Pose2d>): Array<QuinticHermiteSpline> {
        return Array(waypoints.size - 1) { i ->
            val p0 = waypoints[i]
            val p1 = waypoints[i + 1]

            val scalar = 1.2 * p0.translation.getDistance(p1.translation)

            val controlVecA = quinticControlVector(scalar, p0)
            val controlVecB = quinticControlVector(scalar, p1)

            QuinticHermiteSpline(controlVecA.x, controlVecB.x, controlVecA.y, controlVecB.y)
        }
    }

    /**
     * Cubic splines through [start]/[waypoints]/[end], choosing the interior waypoints' headings
     * automatically for continuous curvature (solving a tridiagonal system via [thomasAlgorithm]).
     */
    fun getCubicSplinesFromControlVectors(
        start: Spline.ControlVector,
        waypoints: Array<Translation2d>,
        end: Spline.ControlVector,
    ): Array<CubicHermiteSpline> {
        val splines = arrayOfNulls<CubicHermiteSpline>(waypoints.size + 1)

        val xInitial = start.x
        val yInitial = start.y
        val xFinal = end.x
        val yFinal = end.y

        if (waypoints.size > 1) {
            val newWaypoints = arrayOfNulls<Translation2d>(waypoints.size + 2)
            newWaypoints[0] = Translation2d(xInitial[0], yInitial[0])
            waypoints.copyInto(newWaypoints, 1)
            newWaypoints[newWaypoints.size - 1] = Translation2d(xFinal[0], yFinal[0])

            val n = newWaypoints.size - 2
            val a = DoubleArray(n)
            val b = DoubleArray(n) { 4.0 }
            val c = DoubleArray(n)
            val dx = DoubleArray(n)
            val dy = DoubleArray(n)
            val fx = DoubleArray(n)
            val fy = DoubleArray(n)

            a[0] = 0.0
            for (i in 0 until newWaypoints.size - 3) {
                a[i + 1] = 1.0
                c[i] = 1.0
            }
            c[c.size - 1] = 0.0

            dx[0] = 3 * (newWaypoints[2]!!.x - newWaypoints[0]!!.x) - xInitial[1]
            dy[0] = 3 * (newWaypoints[2]!!.y - newWaypoints[0]!!.y) - yInitial[1]

            if (newWaypoints.size > 4) {
                for (i in 1..newWaypoints.size - 4) {
                    dx[i] = 3 * (newWaypoints[i + 2]!!.x - newWaypoints[i]!!.x)
                    dy[i] = 3 * (newWaypoints[i + 2]!!.y - newWaypoints[i]!!.y)
                }
            }

            dx[dx.size - 1] = 3 * (newWaypoints[newWaypoints.size - 1]!!.x - newWaypoints[newWaypoints.size - 3]!!.x) - xFinal[1]
            dy[dy.size - 1] = 3 * (newWaypoints[newWaypoints.size - 1]!!.y - newWaypoints[newWaypoints.size - 3]!!.y) - yFinal[1]

            thomasAlgorithm(a, b, c, dx, fx)
            thomasAlgorithm(a, b, c, dy, fy)

            val newFx = DoubleArray(fx.size + 2)
            val newFy = DoubleArray(fy.size + 2)
            newFx[0] = xInitial[1]
            newFy[0] = yInitial[1]
            fx.copyInto(newFx, 1)
            fy.copyInto(newFy, 1)
            newFx[newFx.size - 1] = xFinal[1]
            newFy[newFy.size - 1] = yFinal[1]

            for (i in 0 until newFx.size - 1) {
                splines[i] = CubicHermiteSpline(
                    doubleArrayOf(newWaypoints[i]!!.x, newFx[i]),
                    doubleArrayOf(newWaypoints[i + 1]!!.x, newFx[i + 1]),
                    doubleArrayOf(newWaypoints[i]!!.y, newFy[i]),
                    doubleArrayOf(newWaypoints[i + 1]!!.y, newFy[i + 1]),
                )
            }
        } else if (waypoints.size == 1) {
            val xDeriv = (3 * (xFinal[0] - xInitial[0]) - xFinal[1] - xInitial[1]) / 4.0
            val yDeriv = (3 * (yFinal[0] - yInitial[0]) - yFinal[1] - yInitial[1]) / 4.0

            val midX = doubleArrayOf(waypoints[0].x, xDeriv)
            val midY = doubleArrayOf(waypoints[0].y, yDeriv)

            splines[0] = CubicHermiteSpline(xInitial, midX, yInitial, midY)
            splines[1] = CubicHermiteSpline(midX, xFinal, midY, yFinal)
        } else {
            splines[0] = CubicHermiteSpline(xInitial, xFinal, yInitial, yFinal)
        }

        @Suppress("UNCHECKED_CAST")
        return splines as Array<CubicHermiteSpline>
    }

    fun getQuinticSplinesFromControlVectors(controlVectors: Array<Spline.ControlVector>): Array<QuinticHermiteSpline> {
        return Array(controlVectors.size - 1) { i ->
            QuinticHermiteSpline(controlVectors[i].x, controlVectors[i + 1].x, controlVectors[i].y, controlVectors[i + 1].y)
        }
    }

    /**
     * Nudges [splines]' curvature at each shared knot point towards a weighted average, minimizing
     * the integral of the second derivative's absolute value across the whole path (section 4.1.2
     * of Sprunk 2008, "Planning Motion Trajectories for Mobile Robots Using Splines").
     */
    fun optimizeCurvature(splines: Array<QuinticHermiteSpline>): Array<QuinticHermiteSpline> {
        if (splines.size < 2) return splines

        val optimized = arrayOfNulls<QuinticHermiteSpline>(splines.size)
        for (i in 0 until splines.size - 1) {
            val a = splines[i]
            val b = splines[i + 1]

            val aInitial = a.initialControlVector()
            val aFinal = a.finalControlVector()
            val bInitial = b.initialControlVector()
            val bFinal = b.finalControlVector()

            val ca = CubicHermiteSpline(aInitial.x, aFinal.x, aInitial.y, aFinal.y)
            val cb = CubicHermiteSpline(bInitial.x, bFinal.x, bInitial.y, bFinal.y)

            val ddxA = ca.coefficients()[4, 0] + ca.coefficients()[4, 1] + ca.coefficients()[4, 2] + ca.coefficients()[4, 3]
            val ddyA = ca.coefficients()[5, 0] + ca.coefficients()[5, 1] + ca.coefficients()[5, 2] + ca.coefficients()[5, 3]
            val ddxB = cb.coefficients()[4, 1]
            val ddyB = cb.coefficients()[5, 1]

            val dAB = hypot(aFinal.x[0] - aInitial.x[0], aFinal.y[0] - aInitial.y[0])
            val dBC = hypot(bFinal.x[0] - bInitial.x[0], bFinal.y[0] - bInitial.y[0])
            val alpha = dBC / (dAB + dBC)
            val beta = dAB / (dAB + dBC)

            val ddx = alpha * ddxA + beta * ddxB
            val ddy = alpha * ddyA + beta * ddyB

            optimized[i] = QuinticHermiteSpline(
                aInitial.x, doubleArrayOf(aFinal.x[0], aFinal.x[1], ddx),
                aInitial.y, doubleArrayOf(aFinal.y[0], aFinal.y[1], ddy),
            )
            optimized[i + 1] = QuinticHermiteSpline(
                doubleArrayOf(bInitial.x[0], bInitial.x[1], ddx), bFinal.x,
                doubleArrayOf(bInitial.y[0], bInitial.y[1], ddy), bFinal.y,
            )
        }

        @Suppress("UNCHECKED_CAST")
        return optimized as Array<QuinticHermiteSpline>
    }

    /** Solves a tridiagonal system `A*f = d` (above-diagonal [a], diagonal [b], below-diagonal [c]) into [solutionVector]. */
    private fun thomasAlgorithm(a: DoubleArray, b: DoubleArray, c: DoubleArray, d: DoubleArray, solutionVector: DoubleArray) {
        val n = d.size
        val cStar = DoubleArray(n)
        val dStar = DoubleArray(n)

        cStar[0] = c[0] / b[0]
        dStar[0] = d[0] / b[0]

        for (i in 1 until n) {
            val m = 1.0 / (b[i] - a[i] * cStar[i - 1])
            cStar[i] = c[i] * m
            dStar[i] = (d[i] - a[i] * dStar[i - 1]) * m
        }
        solutionVector[n - 1] = dStar[n - 1]
        for (i in n - 2 downTo 0) {
            solutionVector[i] = dStar[i] - cStar[i] * solutionVector[i + 1]
        }
    }

    private fun cubicControlVector(scalar: Double, point: Pose2d) = Spline.ControlVector(
        doubleArrayOf(point.x, scalar * point.rotation.cos),
        doubleArrayOf(point.y, scalar * point.rotation.sin),
    )

    private fun quinticControlVector(scalar: Double, point: Pose2d) = Spline.ControlVector(
        doubleArrayOf(point.x, scalar * point.rotation.cos, 0.0),
        doubleArrayOf(point.y, scalar * point.rotation.sin, 0.0),
    )
}
