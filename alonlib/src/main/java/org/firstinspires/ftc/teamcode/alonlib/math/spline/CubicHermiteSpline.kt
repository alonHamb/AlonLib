package org.firstinspires.ftc.teamcode.alonlib.math.spline

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/**
 * A degree-3 (cubic) Hermite spline -- interpolates between two points given each one's position
 * and velocity (1st derivative). Cheaper than [QuinticHermiteSpline] but can't also match
 * acceleration/curvature at the endpoints.
 */
class CubicHermiteSpline(
    val xInitialControlVector: DoubleArray,
    val xFinalControlVector: DoubleArray,
    val yInitialControlVector: DoubleArray,
    val yFinalControlVector: DoubleArray,
) : Spline(3) {

    private val coefficients: Matrix
    private val initial = ControlVector(xInitialControlVector, yInitialControlVector)
    private val finalVector = ControlVector(xFinalControlVector, yFinalControlVector)

    init {
        val hermite = hermiteBasis
        val x = controlVectorMatrix(xInitialControlVector, xFinalControlVector)
        val y = controlVectorMatrix(yInitialControlVector, yFinalControlVector)

        val xCoeffs = (hermite * x).transpose()
        val yCoeffs = (hermite * y).transpose()

        coefficients = Matrix(6, 4)
        for (i in 0 until 4) {
            coefficients[0, i] = xCoeffs[0, i]
            coefficients[1, i] = yCoeffs[0, i]

            // Rows 2-3: first derivative (power rule); rows 4-5 (below): second derivative.
            coefficients[2, i] = coefficients[0, i] * (3 - i)
            coefficients[3, i] = coefficients[1, i] * (3 - i)
        }
        for (i in 0 until 3) {
            coefficients[4, i] = coefficients[2, i] * (2 - i)
            coefficients[5, i] = coefficients[3, i] * (2 - i)
        }
    }

    override fun coefficients() = coefficients
    override fun initialControlVector() = initial
    override fun finalControlVector() = finalVector

    companion object {
        // Solves P(t) = a3*t^3 + a2*t^2 + a1*t + a0 for a0..a3 given P(0), P'(0), P(1), P'(1).
        private val hermiteBasis = Matrix.fill(
            4, 4,
            +2.0, +1.0, -2.0, +1.0,
            -3.0, -2.0, +3.0, -1.0,
            +0.0, +1.0, +0.0, +0.0,
            +1.0, +0.0, +0.0, +0.0,
        )

        private fun controlVectorMatrix(initial: DoubleArray, terminal: DoubleArray): Matrix {
            if (initial.size < 2 || terminal.size < 2) {
                robotPrintError("cubic Hermite spline control vectors must have size >= 2")
                return Matrix(4, 1)
            }
            return Matrix.vector(initial[0], initial[1], terminal[0], terminal[1])
        }
    }
}
