package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.Discretization
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.math.system.NumericalIntegration
import org.firstinspires.ftc.teamcode.alonlib.math.system.NumericalJacobian
import org.firstinspires.ftc.teamcode.alonlib.math.system.StateSpaceUtil

/**
 * A Kalman filter for nonlinear plant/measurement models `x' = f(x, u)`/`y = h(x, u)`: propagates
 * the error covariance using sigma points ([MerweScaledSigmaPoints]) chosen to approximate the
 * true probability distribution, rather than [ExtendedKalmanFilter]'s linearization.
 *
 * Upstream WPILib implements this as a square-root-form filter (SR-UKF, tracking a Cholesky
 * factor `S` instead of `P` directly, via QR decomposition and rank-1 covariance updates) for
 * better numerical conditioning. This port uses the classical (non-square-root) covariance-form
 * UKF instead -- mathematically equivalent, and implementable with this port's existing
 * inverse/solve/multiply [Matrix] primitives rather than a from-scratch Householder QR and
 * rank-1 Cholesky update/downdate, at the cost of the SR-UKF's extra numerical robustness.
 */
class UnscentedKalmanFilter(
    private val states: Int,
    private val outputs: Int,
    private val f: (Matrix, Matrix) -> Matrix,
    private val h: (Matrix, Matrix) -> Matrix,
    stateStdDevs: Matrix,
    measurementStdDevs: Matrix,
    private val meanFuncX: (Matrix, Matrix) -> Matrix = { sigmas, wm -> weightedMean(sigmas, wm) },
    private val meanFuncY: (Matrix, Matrix) -> Matrix = { sigmas, wm -> weightedMean(sigmas, wm) },
    private val residualFuncX: (Matrix, Matrix) -> Matrix = Matrix::minus,
    private val residualFuncY: (Matrix, Matrix) -> Matrix = Matrix::minus,
    private val addFuncX: (Matrix, Matrix) -> Matrix = Matrix::plus,
    private var dtSeconds: Double,
) : KalmanTypeFilter {

    private val contQ = StateSpaceUtil.makeCovarianceMatrix(stateStdDevs)
    private val contR = StateSpaceUtil.makeCovarianceMatrix(measurementStdDevs)
    private val pts = MerweScaledSigmaPoints(states)

    override var xHat: Matrix = Matrix(states, 1)
    override var p: Matrix = Matrix(states, states)
    private var sigmasF: Matrix = Matrix(states, pts.numSigmas)

    override fun reset() {
        xHat = Matrix(states, 1)
        p = Matrix(states, states)
        sigmasF = Matrix(states, pts.numSigmas)
    }

    override fun predict(u: Matrix, dtSeconds: Double) {
        val sigmas = pts.sigmaPoints(xHat, p)

        sigmasF = Matrix(states, pts.numSigmas)
        for (i in 0 until pts.numSigmas) {
            sigmasF.setColumn(i, NumericalIntegration.rk4(f, sigmas.column(i), u, dtSeconds))
        }

        // Only used to properly discretize Q (van Loan's method) -- the sigma points/RK4
        // integration above already handle the actual nonlinear state propagation.
        val contA = NumericalJacobian.numericalJacobianX(states, states, f, xHat, u)
        val discQ = Discretization.discretizeAQ(contA, contQ, dtSeconds).second
        val (mean, cov) = unscentedTransform(sigmasF, pts.wm, pts.wc, meanFuncX, residualFuncX, discQ)
        xHat = mean
        p = cov

        this.dtSeconds = dtSeconds
    }

    override fun correct(u: Matrix, y: Matrix) = correct(outputs, u, y, h, contR, meanFuncY, residualFuncY, residualFuncX, addFuncX)

    /** As [correct], but for a one-off measurement noise covariance [r]. */
    fun correct(u: Matrix, y: Matrix, r: Matrix) = correct(outputs, u, y, h, r, meanFuncY, residualFuncY, residualFuncX, addFuncX)

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
        meanFuncY: (Matrix, Matrix) -> Matrix = { sigmas, wm -> weightedMean(sigmas, wm) },
        residualFuncY: (Matrix, Matrix) -> Matrix = Matrix::minus,
        residualFuncX: (Matrix, Matrix) -> Matrix = Matrix::minus,
        addFuncX: (Matrix, Matrix) -> Matrix = Matrix::plus,
    ) {
        val discR = Discretization.discretizeR(r, dtSeconds)

        val sigmas = pts.sigmaPoints(xHat, p)
        val sigmasH = Matrix(rows, pts.numSigmas)
        for (i in 0 until pts.numSigmas) sigmasH.setColumn(i, h(sigmas.column(i), u))

        val (yHat, sy) = unscentedTransform(sigmasH, pts.wm, pts.wc, meanFuncY, residualFuncY, discR)

        // Cross covariance of the predicted state and measurement sigma points.
        var pxy = Matrix(states, rows)
        for (i in 0 until pts.numSigmas) {
            val dx = residualFuncX(sigmasF.column(i), xHat)
            val dy = residualFuncY(sigmasH.column(i), yHat)
            pxy += (dx * dy.transpose()) * pts.wc[i, 0]
        }

        // K = P_xy * S_y⁻¹, solved as Kᵀ = S_yᵀ.solve(P_xyᵀ) to avoid an explicit inverse.
        val k = sy.transpose().solve(pxy.transpose()).transpose()

        xHat = addFuncX(xHat, k * residualFuncY(y, yHat))
        p -= k * sy * k.transpose()
    }

    companion object {
        private fun weightedMean(sigmas: Matrix, weights: Matrix): Matrix {
            var mean = Matrix(sigmas.rows, 1)
            for (i in 0 until sigmas.cols) mean += sigmas.column(i) * weights[i, 0]
            return mean
        }

        private fun unscentedTransform(
            sigmas: Matrix,
            wm: Matrix,
            wc: Matrix,
            meanFunc: (Matrix, Matrix) -> Matrix,
            residualFunc: (Matrix, Matrix) -> Matrix,
            noiseCov: Matrix,
        ): Pair<Matrix, Matrix> {
            val mean = meanFunc(sigmas, wm)

            var cov = noiseCov.copy()
            for (i in 0 until sigmas.cols) {
                val d = residualFunc(sigmas.column(i), mean)
                cov += (d * d.transpose()) * wc[i, 0]
            }
            return mean to cov
        }
    }
}
