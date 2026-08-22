package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.system.Discretization
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LinearQuadraticRegulatorTest {

    @Test
    fun `driving a double integrator toward a nonzero reference converges to it`() {
        // Double integrator: position/velocity states, acceleration (voltage-ish) input.
        val a = Matrix.fill(2, 2, 0.0, 1.0, 0.0, 0.0)
        val b = Matrix.vector(0.0, 1.0)
        val q = Matrix.eye(2)
        val r = Matrix.vector(1.0)
        val dt = 0.02

        val lqr = LinearQuadraticRegulator(a, b, q, r, dt)
        val (discA, discB) = Discretization.discretizeAB(a, b, dt)

        var x = Matrix.vector(0.0, 0.0)
        val reference = Matrix.vector(1.0, 0.0)
        repeat(2000) {
            val u = lqr.calculate(x, reference)
            x = discA * x + discB * u
        }

        assertEquals(1.0, x[0, 0], 1e-3)
        assertEquals(0.0, x[1, 0], 1e-3)
    }

    @Test
    fun `closed-loop system is stable (eigenvalues of A-BK stay inside the unit circle)`() {
        val a = Matrix.fill(2, 2, 1.0, 0.02, 0.0, 1.0)
        val b = Matrix.vector(0.0, 0.02)
        val q = Matrix.eye(2)
        val r = Matrix.vector(1.0)

        val lqr = LinearQuadraticRegulator(a, b, q, r, 0.02)
        val closedLoop = a - b * lqr.k

        // A conservative stability proxy that doesn't need eigenvalues: repeated application of a
        // stable discrete system's dynamics must not blow up.
        var x = Matrix.vector(1.0, 1.0)
        repeat(500) { x = closedLoop * x }
        assertTrue(x.normF() < 1.0)
    }
}
