package org.firstinspires.ftc.teamcode.alonlib.math.system

/**
 * Solves the discrete-time algebraic Riccati equation `AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0`
 * for its unique stabilizing solution X, via the Structure-preserving Doubling Algorithm (SDA;
 * Chu/Fan/Lin 2004, the same underlying method WPILib's own JNI-backed "Drake" solver uses).
 * Upstream WPILib calls into a JNI binding to Drake's C++ Eigen-based solver here, which isn't
 * portable to Android -- this reimplements the doubling iteration directly in pure Kotlin/EJML.
 *
 * Preconditions (unchecked, matching WPILib's `dareNoPrecond` -- the caller is responsible for
 * them): Q symmetric positive semidefinite, R symmetric positive definite, (A, B) stabilizable,
 * (A, C) where Q = CᵀC detectable.
 */
object DARE {

    private const val MAX_ITERATIONS = 100
    private const val TOLERANCE = 1e-10

    fun solve(A: Matrix, B: Matrix, Q: Matrix, R: Matrix): Matrix {
        val G = B * R.inverse() * B.transpose()
        return solve(A, G, Q)
    }

    /** The cross-term overload: `AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0`. */
    fun solve(A: Matrix, B: Matrix, Q: Matrix, R: Matrix, N: Matrix): Matrix {
        val rInvNt = R.solve(N.transpose())
        val scrA = A - B * rInvNt
        val scrQ = Q - N * rInvNt
        return solve(scrA, B, scrQ, R)
    }

    private fun solve(A: Matrix, G: Matrix, Q: Matrix): Matrix {
        val n = A.rows
        var Ak = A
        var Gk = G
        var Hk = Q

        repeat(MAX_ITERATIONS) {
            val W = (Matrix.eye(n) + Gk * Hk).inverse()
            val AkW = Ak * W
            val nextA = AkW * Ak
            val nextG = Gk + AkW * Gk * Ak.transpose()
            val nextH = Hk + Ak.transpose() * Hk * W * Ak

            val converged = (nextH - Hk).normF() < TOLERANCE
            Ak = nextA
            Gk = nextG
            Hk = nextH
            if (converged) return Hk
        }
        return Hk
    }
}
