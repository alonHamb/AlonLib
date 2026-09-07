package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.system.DARE
import org.firstinspires.ftc.teamcode.alonlib.math.system.Discretization
import org.firstinspires.ftc.teamcode.alonlib.math.system.LinearSystem
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.math.system.StateSpaceUtil

/**
 * A linear-quadratic regulator: the feedback control law `u = K(r - x)` that minimizes
 * `sum((xᵀQx + uᵀRu) * dt)` subject to `x' = Ax + Bu`. See
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf for the underlying math.
 *
 * Upstream WPILib also has a raw-`Matrix` overload of the per-tolerance constructor (as opposed
 * to the [LinearSystem] one below) -- dropped here since, without WPILib's separate `Vector`
 * type, it would have the exact same erased signature as the cost-matrix constructor and can't
 * coexist with it; call [org.firstinspires.ftc.teamcode.alonlib.math.system.StateSpaceUtil.makeCostMatrix] yourself instead.
 */
class LinearQuadraticRegulator private constructor(val k: Matrix, stateCount: Int, inputCount: Int) {

    var r: Matrix = Matrix(stateCount, 1)
        private set

    var u: Matrix = Matrix(inputCount, 1)
        private set

    constructor(plant: LinearSystem, qElms: Matrix, rElms: Matrix, dtSeconds: Double) :
            this(plant.a, plant.b, StateSpaceUtil.makeCostMatrix(qElms), StateSpaceUtil.makeCostMatrix(rElms), dtSeconds)

    /** Constructs an LQR from continuous-time system matrices [a]/[b] and cost matrices [q]/[r]. */
    constructor(a: Matrix, b: Matrix, q: Matrix, r: Matrix, dtSeconds: Double) : this(
        run {
            val (discA, discB) = Discretization.discretizeAB(a, b, dtSeconds)
            val s = DARE.solve(discA, discB, q, r)
            // K = (BᵀSB + R)⁻¹BᵀSA
            (discB.transpose() * s * discB + r).solve(discB.transpose() * s * discA)
        },
        a.rows,
        b.cols,
    )

    /** Constructs an LQR with a state-input cross-term cost matrix [n]. */
    constructor(a: Matrix, b: Matrix, q: Matrix, r: Matrix, n: Matrix, dtSeconds: Double) : this(
        run {
            val (discA, discB) = Discretization.discretizeAB(a, b, dtSeconds)
            val s = DARE.solve(discA, discB, q, r, n)
            // K = (BᵀSB + R)⁻¹(BᵀSA + Nᵀ)
            (discB.transpose() * s * discB + r).solve(discB.transpose() * s * discA + n.transpose())
        },
        a.rows,
        b.cols,
    )

    /** Zeroes the reference r and output u. */
    fun reset() {
        r.fill(0.0)
        u.fill(0.0)
    }

    /** The next controller output for the current state [x], tracking the existing reference [r]. */
    fun calculate(x: Matrix): Matrix {
        u = k * (r - x)
        return u
    }

    /** The next controller output for the current state [x], tracking a new reference [nextR]. */
    fun calculate(x: Matrix, nextR: Matrix): Matrix {
        r = nextR
        return calculate(x)
    }
}
