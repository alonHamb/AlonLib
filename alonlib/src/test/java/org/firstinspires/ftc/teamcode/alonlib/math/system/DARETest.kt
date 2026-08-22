package org.firstinspires.ftc.teamcode.alonlib.math.system

import org.junit.Assert.assertEquals
import org.junit.Test

class DARETest {

    private val delta = 1e-6

    @Test
    fun `scalar DARE matches the closed-form golden ratio solution`() {
        // AᵀXA - X - AᵀXB(BᵀXB+R)⁻¹BᵀXA + Q = 0 with A=B=Q=R=1 reduces to X² - X - 1 = 0,
        // whose positive root is the golden ratio.
        val a = Matrix.vector(1.0)
        val b = Matrix.vector(1.0)
        val q = Matrix.vector(1.0)
        val r = Matrix.vector(1.0)

        val x = DARE.solve(a, b, q, r)

        assertEquals((1.0 + kotlin.math.sqrt(5.0)) / 2.0, x[0, 0], delta)
    }

    @Test
    fun `solution satisfies the Riccati equation residual`() {
        val a = Matrix.fill(2, 2, 1.0, 0.1, 0.0, 1.0)
        val b = Matrix.vector(0.0, 1.0)
        val q = Matrix.eye(2)
        val r = Matrix.vector(1.0)

        val x = DARE.solve(a, b, q, r)

        val residual = a.transpose() * x * a - x -
                (a.transpose() * x * b) * (b.transpose() * x * b + r).inverse() * (b.transpose() * x * a) + q

        assertEquals(0.0, residual.normF(), 1e-6)
    }
}
