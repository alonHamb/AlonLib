package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.junit.Assert.assertEquals
import org.junit.Test

class UnscentedKalmanFilterTest {

    @Test
    fun `converges toward repeated noisy measurements of a static state`() {
        val f = { x: Matrix, _: Matrix -> Matrix(1, 1) }
        val h = { x: Matrix, _: Matrix -> x.copy() }
        val filter = UnscentedKalmanFilter(1, 1, f, h, Matrix.vector(1.0), Matrix.vector(0.3), dtSeconds = 0.02)

        val u = Matrix.vector(0.0)
        val truth = 5.0
        repeat(500) {
            filter.predict(u, 0.02)
            val noisy = truth + if (it % 2 == 0) 0.2 else -0.2
            filter.correct(u, Matrix.vector(noisy))
        }

        assertEquals(truth, filter.xHat[0, 0], 0.05)
    }

    @Test
    fun `predicting with a two-state system keeps dimensions consistent`() {
        // Double integrator: dx/dt = [v, 0] (zero input/acceleration for this smoke test).
        val f = { x: Matrix, _: Matrix -> Matrix.vector(x[1, 0], 0.0) }
        val h = { x: Matrix, _: Matrix -> Matrix.vector(x[0, 0]) }
        val filter = UnscentedKalmanFilter(2, 1, f, h, Matrix.vector(0.01, 0.01), Matrix.vector(0.1), dtSeconds = 0.02)

        filter.xHat = Matrix.vector(0.0, 1.0)
        filter.predict(Matrix(0, 1), 0.02)

        assertEquals(0.02, filter.xHat[0, 0], 1e-3)
        assertEquals(2, filter.xHat.rows)
    }
}
