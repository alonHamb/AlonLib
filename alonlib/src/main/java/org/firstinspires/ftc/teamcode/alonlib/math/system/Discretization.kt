package org.firstinspires.ftc.teamcode.alonlib.math.system

/** Converts continuous-time state-space matrices to their discrete-time equivalents. */
object Discretization {

    /** `A_d = e^(A*dt)`. */
    fun discretizeA(contA: Matrix, dtSeconds: Double) = (contA * dtSeconds).exp()

    /** The discretized (A, B) pair, via the augmented-matrix exponential trick. */
    fun discretizeAB(contA: Matrix, contB: Matrix, dtSeconds: Double): Pair<Matrix, Matrix> {
        val states = contA.rows
        val inputs = contB.cols

        // M = [A B]   phi = e^(M*dt) = [A_d  B_d]
        //     [0 0]                    [ 0    I ]
        val m = Matrix(states + inputs, states + inputs)
        m.assignBlock(0, 0, contA)
        m.assignBlock(0, contA.cols, contB)

        val phi = (m * dtSeconds).exp()

        val discA = phi.block(states, states, 0, 0)
        val discB = phi.block(states, inputs, 0, contB.rows)
        return discA to discB
    }

    /** The discretized (A, Q) pair for propagating process-noise covariance. */
    fun discretizeAQ(contA: Matrix, contQ: Matrix, dtSeconds: Double): Pair<Matrix, Matrix> {
        val states = contA.rows
        val q = (contQ + contQ.transpose()) / 2.0

        // M = [-A  Q ]   phi = e^(M*dt) = [-A_d  A_d⁻¹Q_d]
        //     [ 0  Aᵀ]                    [  0     A_dᵀ  ]
        val m = Matrix(2 * states, 2 * states)
        m.assignBlock(0, 0, contA * -1.0)
        m.assignBlock(0, states, q)
        m.assignBlock(states, states, contA.transpose())

        val phi = (m * dtSeconds).exp()

        val phi12 = phi.block(states, states, 0, states)
        val phi22 = phi.block(states, states, states, states)

        val discA = phi22.transpose()
        val discQUnsymmetrized = discA * phi12
        val discQ = (discQUnsymmetrized + discQUnsymmetrized.transpose()) / 2.0
        return discA to discQ
    }

    /** `R_d = R / dt`. Note `dt = 0` divides by zero. */
    fun discretizeR(contR: Matrix, dtSeconds: Double) = contR / dtSeconds
}
