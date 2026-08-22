package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix

/** Common interface for [KalmanFilter], [ExtendedKalmanFilter], and [UnscentedKalmanFilter], so [KalmanFilterLatencyCompensator] can work with any of them. */
interface KalmanTypeFilter {

    /** The error covariance matrix. */
    var p: Matrix

    /** The state estimate. */
    var xHat: Matrix

    fun reset()

    /** Projects the model into the future with control input [u] over [dtSeconds]. */
    fun predict(u: Matrix, dtSeconds: Double)

    /** Corrects the state estimate using measurement [y], given the same [u] used in [predict]. */
    fun correct(u: Matrix, y: Matrix)
}
