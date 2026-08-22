package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.DARE
import org.firstinspires.ftc.teamcode.alonlib.math.system.Discretization
import org.firstinspires.ftc.teamcode.alonlib.math.system.LinearSystem
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.math.system.StateSpaceUtil

/**
 * A (linear) Kalman filter: combines a [plant] model's predictions with noisy measurements to
 * estimate a system's true state, weighting each by how much they're trusted (via the steady-state
 * Kalman gain, found by solving a [DARE]). See
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9 for the underlying math.
 */
class KalmanFilter(
    private val states: Int,
    private val plant: LinearSystem,
    stateStdDevs: Matrix,
    measurementStdDevs: Matrix,
    private var dtSeconds: Double,
) : KalmanTypeFilter {

    private val contQ = StateSpaceUtil.makeCovarianceMatrix(stateStdDevs)
    private val contR = StateSpaceUtil.makeCovarianceMatrix(measurementStdDevs)

    private val initP: Matrix = run {
        val (discA, discQ) = Discretization.discretizeAQ(plant.a, contQ, dtSeconds)
        val discR = Discretization.discretizeR(contR, dtSeconds)
        DARE.solve(discA.transpose(), plant.c.transpose(), discQ, discR)
    }

    override var xHat: Matrix = Matrix(states, 1)
    override var p: Matrix = initP

    override fun reset() {
        xHat = Matrix(states, 1)
        p = initP
    }

    override fun predict(u: Matrix, dtSeconds: Double) {
        val (discA, discQ) = Discretization.discretizeAQ(plant.a, contQ, dtSeconds)

        xHat = plant.calculateX(xHat, u, dtSeconds)

        // Pₖ₊₁⁻ = APₖ⁻Aᵀ + Q
        p = discA * p * discA.transpose() + discQ

        this.dtSeconds = dtSeconds
    }

    override fun correct(u: Matrix, y: Matrix) = correct(u, y, contR)

    /** As [correct], but for a one-off measurement noise covariance [r] different from the one this filter was constructed with. */
    fun correct(u: Matrix, y: Matrix, r: Matrix) {
        val c = plant.c
        val d = plant.d
        val discR = Discretization.discretizeR(r, dtSeconds)

        val s = c * p * c.transpose() + discR

        // K = PCᵀS⁻¹, solved as Kᵀ = Sᵀ.solve(CPᵀ) to avoid an explicit inverse.
        val k = s.transpose().solve(c * p.transpose()).transpose()

        // x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − (Cx̂ₖ₊₁⁻ + Duₖ₊₁))
        xHat += k * (y - (c * xHat + d * u))

        // Pₖ₊₁⁺ = (I−KC)Pₖ₊₁⁻(I−KC)ᵀ + KRKᵀ -- Joseph form, for numerical stability.
        val iMinusKC = Matrix.eye(states) - k * c
        p = iMinusKC * p * iMinusKC.transpose() + k * discR * k.transpose()
    }
}
