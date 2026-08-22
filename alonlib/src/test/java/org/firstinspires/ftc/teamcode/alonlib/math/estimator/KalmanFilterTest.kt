package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.LinearSystem
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.junit.Assert.assertEquals
import org.junit.Test

class KalmanFilterTest {

    @Test
    fun `converges toward repeated noisy measurements of a static state`() {
        // A single stationary state directly observed: x' = 0, y = x.
        val plant = LinearSystem(Matrix.vector(0.0), Matrix.vector(0.0), Matrix.vector(1.0), Matrix.vector(0.0))
        val filter = KalmanFilter(1, plant, Matrix.vector(1.0), Matrix.vector(0.3), 0.02)

        val u = Matrix.vector(0.0)
        val truth = 5.0
        repeat(500) {
            filter.predict(u, 0.02)
            // Deterministic "noise": alternate slightly above/below the true value.
            val noisy = truth + if (it % 2 == 0) 0.2 else -0.2
            filter.correct(u, Matrix.vector(noisy))
        }

        assertEquals(truth, filter.xHat[0, 0], 0.05)
    }

    @Test
    fun `reset restores the initial state estimate and covariance`() {
        val plant = LinearSystem(Matrix.vector(0.0), Matrix.vector(0.0), Matrix.vector(1.0), Matrix.vector(0.0))
        val filter = KalmanFilter(1, plant, Matrix.vector(0.01), Matrix.vector(1.0), 0.02)
        val initialP = filter.p[0, 0]

        filter.predict(Matrix.vector(0.0), 0.02)
        filter.correct(Matrix.vector(0.0), Matrix.vector(10.0))
        filter.reset()

        assertEquals(0.0, filter.xHat[0, 0], 1e-9)
        assertEquals(initialP, filter.p[0, 0], 1e-9)
    }
}
