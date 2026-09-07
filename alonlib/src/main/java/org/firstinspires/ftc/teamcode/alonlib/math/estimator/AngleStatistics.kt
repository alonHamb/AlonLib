package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.angleModulus
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Vector arithmetic for filters (e.g. [UnscentedKalmanFilter]/[ExtendedKalmanFilter]) whose state
 * or measurement vector has an angle component -- angles wrap, so a plain subtract/add/mean would
 * be wrong across the +-pi boundary.
 */
object AngleStatistics {

    /** `a - b`, with the [angleStateIdx] row wrapped to `(-pi, pi]`. */
    fun angleResidual(a: Matrix, b: Matrix, angleStateIdx: Int): Matrix {
        val result = a - b
        result[angleStateIdx, 0] = angleModulus(result[angleStateIdx, 0])
        return result
    }

    fun angleResidual(angleStateIdx: Int): (Matrix, Matrix) -> Matrix = { a, b -> angleResidual(a, b, angleStateIdx) }

    /** `a + b`, with the [angleStateIdx] row wrapped to `(-pi, pi]`. */
    fun angleAdd(a: Matrix, b: Matrix, angleStateIdx: Int): Matrix {
        val result = a + b
        result[angleStateIdx, 0] = angleModulus(result[angleStateIdx, 0])
        return result
    }

    fun angleAdd(angleStateIdx: Int): (Matrix, Matrix) -> Matrix = { a, b -> angleAdd(a, b, angleStateIdx) }

    /** The weighted mean of [sigmas]' columns, with the [angleStateIdx] row averaged circularly (via atan2 of the mean sin/cos). */
    fun angleMean(sigmas: Matrix, wm: Matrix, angleStateIdx: Int): Matrix {
        var sumSin = 0.0
        var sumCos = 0.0
        for (i in 0 until sigmas.cols) {
            val angle = sigmas[angleStateIdx, i]
            sumSin += sin(angle) * wm[i, 0]
            sumCos += cos(angle) * wm[i, 0]
        }

        var result = Matrix(sigmas.rows, 1)
        for (i in 0 until sigmas.cols) result += sigmas.column(i) * wm[i, 0]
        result[angleStateIdx, 0] = atan2(sumSin, sumCos)
        return result
    }

    fun angleMean(angleStateIdx: Int): (Matrix, Matrix) -> Matrix = { sigmas, wm -> angleMean(sigmas, wm, angleStateIdx) }
}
