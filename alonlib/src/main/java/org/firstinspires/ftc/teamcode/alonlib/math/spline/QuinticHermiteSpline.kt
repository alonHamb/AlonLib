package org.firstinspires.ftc.teamcode.alonlib.math.spline

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/**
 * A degree-5 (quintic) Hermite spline -- interpolates between two points given each one's
 * position, velocity, and acceleration (1st and 2nd derivatives). Costs more to evaluate than
 * [CubicHermiteSpline] but produces continuous curvature, which matters for smooth trajectory
 * tracking.
 */
class QuinticHermiteSpline(
    val xInitialControlVector: DoubleArray,
    val xFinalControlVector: DoubleArray,
    val yInitialControlVector: DoubleArray,
    val yFinalControlVector: DoubleArray,
) : Spline(5) {

    private val coefficients: Matrix
    private val initial = ControlVector(xInitialControlVector, yInitialControlVector)
    private val finalVector = ControlVector(xFinalControlVector, yFinalControlVector)

    init {
        val hermite = hermiteBasis
        val x = controlVectorMatrix(xInitialControlVector, xFinalControlVector)
        val y = controlVectorMatrix(yInitialControlVector, yFinalControlVector)

        val xCoeffs = (hermite * x).transpose()
        val yCoeffs = (hermite * y).transpose()

        coefficients = Matrix(6, 6)
        for (i in 0 until 6) {
            coefficients[0, i] = xCoeffs[0, i]
            coefficients[1, i] = yCoeffs[0, i]
        }
        for (i in 0 until 6) {
            coefficients[2, i] = coefficients[0, i] * (5 - i)
            coefficients[3, i] = coefficients[1, i] * (5 - i)
        }
        for (i in 0 until 5) {
            coefficients[4, i] = coefficients[2, i] * (4 - i)
            coefficients[5, i] = coefficients[3, i] * (4 - i)
        }
    }

    override fun coefficients() = coefficients
    override fun initialControlVector() = initial
    override fun finalControlVector() = finalVector

    companion object {
        // Solves P(t) = a5*t^5 + ... + a0 for a0..a5 given P(0), P'(0), P"(0), P(1), P'(1), P"(1).
        private val hermiteBasis = Matrix.fill(
            6, 6,
            -6.0, -3.0, -0.5, +6.0, -3.0, +0.5,
            +15.0, +8.0, +1.5, -15.0, +7.0, -1.0,
            -10.0, -6.0, -1.5, +10.0, -4.0, +0.5,
            +0.0, +0.0, +0.5, +0.0, +0.0, +0.0,
            +0.0, +1.0, +0.0, +0.0, +0.0, +0.0,
            +1.0, +0.0, +0.0, +0.0, +0.0, +0.0,
        )

        private fun controlVectorMatrix(initial: DoubleArray, terminal: DoubleArray): Matrix {
            if (initial.size != 3 || terminal.size != 3) {
                robotPrintError("quintic Hermite spline control vectors must have size 3")
                return Matrix(6, 1)
            }
            return Matrix.vector(initial[0], initial[1], initial[2], terminal[0], terminal[1], terminal[2])
        }
    }
}
