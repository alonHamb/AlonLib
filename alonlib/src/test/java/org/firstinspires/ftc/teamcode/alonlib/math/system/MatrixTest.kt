package org.firstinspires.ftc.teamcode.alonlib.math.system

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class MatrixTest {

    private val delta = 1e-9

    @Test
    fun `fill builds a matrix from row-major values`() {
        val m = Matrix.fill(2, 3, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        assertEquals(2, m.rows)
        assertEquals(3, m.cols)
        assertEquals(1.0, m[0, 0], delta)
        assertEquals(3.0, m[0, 2], delta)
        assertEquals(6.0, m[1, 2], delta)
    }

    @Test
    fun `get and set index by row then column`() {
        val m = Matrix(2, 2)
        m[0, 1] = 5.0
        assertEquals(5.0, m[0, 1], delta)
        assertEquals(0.0, m[1, 0], delta)
    }

    @Test
    fun `plus and minus add and subtract element-wise`() {
        val a = Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0)
        val b = Matrix.fill(2, 2, 5.0, 6.0, 7.0, 8.0)
        assertEquals(Matrix.fill(2, 2, 6.0, 8.0, 10.0, 12.0), a + b)
        assertEquals(Matrix.fill(2, 2, -4.0, -4.0, -4.0, -4.0), a - b)
    }

    @Test
    fun `unaryMinus negates every element`() {
        val m = Matrix.fill(1, 2, 1.0, -2.0)
        assertEquals(Matrix.fill(1, 2, -1.0, 2.0), -m)
    }

    @Test
    fun `times performs matrix multiplication`() {
        val a = Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0)
        val identity = Matrix.eye(2)
        assertEquals(a, a * identity)

        val b = Matrix.fill(2, 1, 1.0, 1.0)
        val result = a * b
        assertEquals(1, result.cols)
        assertEquals(3.0, result[0, 0], delta)
        assertEquals(7.0, result[1, 0], delta)
    }

    @Test
    fun `times and div by a scalar scale every element`() {
        val m = Matrix.fill(1, 2, 2.0, 4.0)
        assertEquals(Matrix.fill(1, 2, 4.0, 8.0), m * 2.0)
        assertEquals(Matrix.fill(1, 2, 1.0, 2.0), m / 2.0)
    }

    @Test
    fun `elementTimes and elementDiv operate component-wise`() {
        val a = Matrix.fill(1, 2, 2.0, 3.0)
        val b = Matrix.fill(1, 2, 4.0, 5.0)
        assertEquals(Matrix.fill(1, 2, 8.0, 15.0), a.elementTimes(b))
        assertEquals(Matrix.fill(1, 2, 2.0, 3.0), (a.elementTimes(b)).elementDiv(b))
    }

    @Test
    fun `transpose flips rows and columns`() {
        val m = Matrix.fill(2, 3, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        val t = m.transpose()
        assertEquals(3, t.rows)
        assertEquals(2, t.cols)
        assertEquals(m[0, 2], t[2, 0], delta)
    }

    @Test
    fun `inverse composed with the original is the identity`() {
        val m = Matrix.fill(2, 2, 4.0, 7.0, 2.0, 6.0)
        val result = m * m.inverse()
        assertEquals(Matrix.eye(2), result)
    }

    @Test
    fun `inverse of a non-square matrix returns zeros and does not throw`() {
        val m = Matrix(2, 3)
        assertEquals(Matrix.zeros(2, 3), m.inverse())
    }

    @Test
    fun `inverse of a singular matrix returns zeros and does not throw`() {
        val m = Matrix.fill(2, 2, 1.0, 2.0, 2.0, 4.0)
        assertEquals(Matrix.zeros(2, 2), m.inverse())
    }

    @Test
    fun `solve finds x such that this times x equals b`() {
        val a = Matrix.fill(2, 2, 2.0, 0.0, 0.0, 4.0)
        val b = Matrix.vector(6.0, 8.0)
        val x = a.solve(b)
        assertEquals(3.0, x[0, 0], delta)
        assertEquals(2.0, x[1, 0], delta)
    }

    @Test
    fun `det computes the determinant`() {
        assertEquals(-2.0, Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0).det(), delta)
        assertEquals(1.0, Matrix.eye(3).det(), delta)
    }

    @Test
    fun `trace sums the diagonal`() {
        assertEquals(5.0, Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0).trace(), delta)
    }

    @Test
    fun `normF is the Frobenius norm`() {
        assertEquals(5.0, Matrix.fill(1, 2, 3.0, 4.0).normF(), delta)
    }

    @Test
    fun `eye builds an identity matrix`() {
        val identity = Matrix.eye(3)
        for (r in 0 until 3) {
            for (c in 0 until 3) {
                assertEquals(if (r == c) 1.0 else 0.0, identity[r, c], delta)
            }
        }
    }

    @Test
    fun `zeros builds a matrix of the given size filled with zero`() {
        val m = Matrix.zeros(2, 3)
        assertEquals(2, m.rows)
        assertEquals(3, m.cols)
        assertEquals(0.0, m[1, 2], delta)
    }

    @Test
    fun `vector builds a column vector`() {
        val v = Matrix.vector(1.0, 2.0, 3.0)
        assertEquals(3, v.rows)
        assertEquals(1, v.cols)
        assertEquals(2.0, v[1, 0], delta)
    }

    @Test
    fun `equals compares values, not reference`() {
        assertEquals(Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0), Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0))
        assertTrue(Matrix.fill(1, 1, 1.0) != Matrix.fill(1, 1, 2.0))
    }

    @Test
    fun `exp of the zero matrix is the identity`() {
        assertEquals(Matrix.eye(3), Matrix(3, 3).exp())
    }

    @Test
    fun `exp of a diagonal matrix exponentiates each diagonal element`() {
        val m = Matrix.fill(2, 2, 1.0, 0.0, 0.0, 2.0)
        val result = m.exp()
        assertEquals(kotlin.math.exp(1.0), result[0, 0], 1e-9)
        assertEquals(kotlin.math.exp(2.0), result[1, 1], 1e-9)
        assertEquals(0.0, result[0, 1], 1e-9)
    }

    @Test
    fun `exp satisfies e^A times e^-A equals identity`() {
        val m = Matrix.fill(2, 2, 0.1, 0.4, -0.3, 0.2)
        val product = m.exp() * (m * -1.0).exp()
        assertTrue((product - Matrix.eye(2)).normF() < 1e-6)
    }

    @Test
    fun `assignBlock and block round-trip a submatrix`() {
        val m = Matrix(3, 3)
        m.assignBlock(1, 1, Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0))
        assertEquals(Matrix.fill(2, 2, 1.0, 2.0, 3.0, 4.0), m.block(2, 2, 1, 1))
        assertEquals(0.0, m[0, 0], delta)
    }

    @Test
    fun `maxAbs finds the largest magnitude element`() {
        assertEquals(5.0, Matrix.fill(1, 3, -5.0, 2.0, 4.0).maxAbs(), delta)
    }
}
