package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.DARE
import org.firstinspires.ftc.teamcode.alonlib.math.system.Discretization
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.math.system.NumericalIntegration
import org.firstinspires.ftc.teamcode.alonlib.math.system.NumericalJacobian
import org.firstinspires.ftc.teamcode.alonlib.math.system.StateSpaceUtil

/**
 * A Kalman filter for nonlinear plant/measurement models `x' = f(x, u)`/`y = h(x, u)`: it
 * linearizes both around the current state estimate (via a numerical Jacobian) at every step, then
 * applies the linear Kalman filter equations. See
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9 for the underlying math.
 *
 * Upstream WPILib's constructor only seeds the initial P via [DARE] when `(A, C)` is detectable
 * (checked through a JNI call this port doesn't have); this always attempts it instead, matching
 * [DARE]'s own unchecked ("no precondition") contract elsewhere in this port.
 */
class ExtendedKalmanFilter(
    private val states: Int,
    inputs: Int,
    private val outputs: Int,
    private val f: (Matrix, Matrix) -> Matrix,
    private val h: (Matrix, Matrix) -> Matrix,
    stateStdDevs: Matrix,
    measurementStdDevs: Matrix,
    private val residualFuncY: (Matrix, Matrix) -> Matrix = Matrix::minus,
    private val addFuncX: (Matrix, Matrix) -> Matrix = Matrix::plus,
    dtSeconds: Double,
) : KalmanTypeFilter {

    private val contQ = StateSpaceUtil.makeCovarianceMatrix(stateStdDevs)
    private val contR = StateSpaceUtil.makeCovarianceMatrix(measurementStdDevs)
    private var dtSeconds = dtSeconds

    private val initP: Matrix = run {
        val zeroInput = Matrix(inputs, 1)
        val contA = NumericalJacobian.numericalJacobianX(states, states, f, Matrix(states, 1), zeroInput)
        val c = NumericalJacobian.numericalJacobianX(outputs, states, h, Matrix(states, 1), zeroInput)
        val (discA, discQ) = Discretization.discretizeAQ(contA, contQ, dtSeconds)
        val discR = Discretization.discretizeR(contR, dtSeconds)
        DARE.solve(discA.transpose(), c.transpose(), discQ, discR)
    }

    override var xHat: Matrix = Matrix(states, 1)
    override var p: Matrix = initP

    override fun reset() {
        xHat = Matrix(states, 1)
        p = initP
    }

    override fun predict(u: Matrix, dtSeconds: Double) = predict(u, f, dtSeconds)

    /** As [predict], but linearizing a different (e.g. simplified) dynamics function [f]. */
    fun predict(u: Matrix, f: (Matrix, Matrix) -> Matrix, dtSeconds: Double) {
        val contA = NumericalJacobian.numericalJacobianX(states, states, f, xHat, u)
        val (discA, discQ) = Discretization.discretizeAQ(contA, contQ, dtSeconds)

        xHat = NumericalIntegration.rk4(f, xHat, u, dtSeconds)

        // Pₖ₊₁⁻ = APₖ⁻Aᵀ + Q
        p = discA * p * discA.transpose() + discQ

        this.dtSeconds = dtSeconds
    }

    override fun correct(u: Matrix, y: Matrix) = correct(outputs, u, y, h, contR, residualFuncY, addFuncX)

    /** As [correct], but for a one-off measurement noise covariance [r]. */
    fun correct(u: Matrix, y: Matrix, r: Matrix) = correct(outputs, u, y, h, r, residualFuncY, addFuncX)

    /**
     * As [correct], but for a different measurement vector shape/function -- lets a single filter
     * mix measurements from several sensors, each with a different `h`/rows/[r].
     */
    fun correct(
        rows: Int,
        u: Matrix,
        y: Matrix,
        h: (Matrix, Matrix) -> Matrix,
        r: Matrix,
        residualFuncY: (Matrix, Matrix) -> Matrix = Matrix::minus,
        addFuncX: (Matrix, Matrix) -> Matrix = Matrix::plus,
    ) {
        val c = NumericalJacobian.numericalJacobianX(rows, states, h, xHat, u)
        val discR = Discretization.discretizeR(r, dtSeconds)

        val s = c * p * c.transpose() + discR

        // K = PCᵀS⁻¹, solved as Kᵀ = Sᵀ.solve(CPᵀ) to avoid an explicit inverse.
        val k = s.transpose().solve(c * p.transpose()).transpose()

        // x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − h(x̂ₖ₊₁⁻, uₖ₊₁))
        xHat = addFuncX(xHat, k * residualFuncY(y, h(xHat, u)))

        // Pₖ₊₁⁺ = (I−KC)Pₖ₊₁⁻(I−KC)ᵀ + KRKᵀ -- Joseph form, for numerical stability.
        val iMinusKC = Matrix.eye(states) - k * c
        p = iMinusKC * p * iMinusKC.transpose() + k * discR * k.transpose()
    }
}
