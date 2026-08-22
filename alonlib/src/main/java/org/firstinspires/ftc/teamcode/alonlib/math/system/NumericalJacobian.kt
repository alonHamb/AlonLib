package org.firstinspires.ftc.teamcode.alonlib.math.system

/** Central-difference numerical Jacobians, used to linearize nonlinear plant/measurement models (e.g. for an Extended Kalman Filter). */
object NumericalJacobian {

    private const val EPSILON = 1e-5

    /** The [rows]x[cols] Jacobian of [f] with respect to its argument, evaluated at [x]. */
    fun numericalJacobian(rows: Int, cols: Int, f: (Matrix) -> Matrix, x: Matrix): Matrix {
        val result = Matrix(rows, cols)

        for (i in 0 until cols) {
            val dxPlus = x.copy()
            val dxMinus = x.copy()
            dxPlus[i, 0] = dxPlus[i, 0] + EPSILON
            dxMinus[i, 0] = dxMinus[i, 0] - EPSILON
            val dF = (f(dxPlus) - f(dxMinus)) / (2 * EPSILON)
            result.setColumn(i, dF)
        }

        return result
    }

    /** The Jacobian of `f(x, u)` with respect to `x`, evaluated at ([x], [u]). */
    fun numericalJacobianX(rows: Int, states: Int, f: (Matrix, Matrix) -> Matrix, x: Matrix, u: Matrix) =
        numericalJacobian(rows, states, { newX -> f(newX, u) }, x)

    /** The Jacobian of `f(x, u)` with respect to `u`, evaluated at ([x], [u]). */
    fun numericalJacobianU(rows: Int, inputs: Int, f: (Matrix, Matrix) -> Matrix, x: Matrix, u: Matrix) =
        numericalJacobian(rows, inputs, { newU -> f(x, newU) }, u)
}
