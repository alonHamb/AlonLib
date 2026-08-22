package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Generates sigma points and weights for [UnscentedKalmanFilter], per Van der Merwe's 2004
 * dissertation ("Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space
 * Models"). `2*states+1` sigma points are generated from a state mean/covariance.
 */
class MerweScaledSigmaPoints(private val states: Int, private val alpha: Double = 1e-3, beta: Double = 2.0, private val kappa: Int = 3 - states) {

    val numSigmas = 2 * states + 1

    /** Weights for computing the sigma points' mean. */
    val wm = Matrix(numSigmas, 1)

    /** Weights for computing the sigma points' covariance. */
    val wc = Matrix(numSigmas, 1)

    init {
        val lambda = alpha.pow(2) * (states + kappa) - states
        val c = 0.5 / (states + lambda)
        wm.fill(c)
        wc.fill(c)
        wm[0, 0] = lambda / (states + lambda)
        wc[0, 0] = lambda / (states + lambda) + (1 - alpha.pow(2) + beta)
    }

    /** The `states x (2*states+1)` sigma points around mean [x] with covariance [p]. */
    fun sigmaPoints(x: Matrix, p: Matrix): Matrix {
        val lambda = alpha.pow(2) * (states + kappa) - states
        val eta = sqrt(lambda + states)
        val u = p.cholesky() * eta

        val sigmas = Matrix(states, numSigmas)
        sigmas.setColumn(0, x)
        for (k in 0 until states) {
            val column = u.column(k)
            sigmas.setColumn(k + 1, x + column)
            sigmas.setColumn(states + k + 1, x - column)
        }
        return sigmas
    }
}
