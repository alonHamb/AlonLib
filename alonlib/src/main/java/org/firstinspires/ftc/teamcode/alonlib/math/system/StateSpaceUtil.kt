package org.firstinspires.ftc.teamcode.alonlib.math.system

import kotlin.random.Random

/**
 * State-space helper functions. Upstream WPILib's `isStabilizable`/`isDetectable` precondition
 * checks are JNI-backed (Eigen eigenvalue decomposition) and not portable here -- skipped, same
 * as this port's [DARE] solver already being the unchecked `dareNoPrecond` variant.
 */
object StateSpaceUtil {

    /** A diagonal covariance matrix for a Kalman filter's Q/R, from a vector of per-state/output standard deviations. */
    fun makeCovarianceMatrix(stdDevs: Matrix): Matrix {
        val result = Matrix(stdDevs.rows, stdDevs.rows)
        for (i in 0 until stdDevs.rows) result[i, i] = stdDevs[i, 0] * stdDevs[i, 0]
        return result
    }

    /** A vector of independent Gaussian white noise, scaled per-element by [stdDevs]. */
    fun makeWhiteNoiseVector(stdDevs: Matrix): Matrix {
        val result = Matrix(stdDevs.rows, 1)
        for (i in 0 until stdDevs.rows) result[i, 0] = Random.nextGaussian() * stdDevs[i, 0]
        return result
    }

    /**
     * A diagonal LQR cost matrix from a vector of per-state/input tolerances (Bryson's rule): the
     * inverse square of each tolerance on the diagonal, or 0 where a tolerance is infinite.
     */
    fun makeCostMatrix(tolerances: Matrix): Matrix {
        val result = Matrix(tolerances.rows, tolerances.rows)
        for (i in 0 until tolerances.rows) {
            result[i, i] = if (tolerances[i, 0] == Double.POSITIVE_INFINITY) 0.0 else 1.0 / (tolerances[i, 0] * tolerances[i, 0])
        }
        return result
    }

    /** Element-wise clamp of [u] between [uMin] and [uMax]. */
    fun clampInputMaxMagnitude(u: Matrix, uMin: Matrix, uMax: Matrix): Matrix {
        val result = Matrix(u.rows, 1)
        for (i in 0 until u.rows) result[i, 0] = u[i, 0].coerceIn(uMin[i, 0], uMax[i, 0])
        return result
    }

    /** Uniformly scales [u] down (preserving direction) if its largest element exceeds [maxMagnitude]. */
    fun desaturateInputVector(u: Matrix, maxMagnitude: Double): Matrix {
        val maxValue = u.maxAbs()
        return if (maxValue > maxMagnitude) u * (maxMagnitude / maxValue) else u
    }
}

private fun Random.nextGaussian(): Double {
    var u1: Double
    var u2: Double
    do {
        u1 = nextDouble()
        u2 = nextDouble()
    } while (u1 <= Double.MIN_VALUE)
    return kotlin.math.sqrt(-2.0 * kotlin.math.ln(u1)) * kotlin.math.cos(2.0 * Math.PI * u2)
}
