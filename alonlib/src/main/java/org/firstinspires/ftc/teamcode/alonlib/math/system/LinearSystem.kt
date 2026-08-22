package org.firstinspires.ftc.teamcode.alonlib.math.system

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/**
 * A plant modeled in state-space notation: `x' = Ax + Bu`, `y = Cx + Du`. See [Models] for common
 * mechanism factories (elevator, arm, flywheel, drivetrain, ...).
 */
class LinearSystem(val a: Matrix, val b: Matrix, val c: Matrix, val d: Matrix) {

    init {
        for (m in listOf(a, b, c, d)) {
            for (r in 0 until m.rows) for (col in 0 until m.cols) {
                if (!m[r, col].isFinite()) {
                    robotPrintError("LinearSystem matrix element isn't finite -- usually a model implementation error")
                }
            }
        }
    }

    /** The next state, given the current [x] and (already-clamped) input [clampedU], over [dtSeconds]. */
    fun calculateX(x: Matrix, clampedU: Matrix, dtSeconds: Double): Matrix {
        val (discA, discB) = Discretization.discretizeAB(a, b, dtSeconds)
        return discA * x + discB * clampedU
    }

    /** The output y for state [x] and (already-clamped) input [clampedU]. */
    fun calculateY(x: Matrix, clampedU: Matrix): Matrix = c * x + d * clampedU

    override fun toString() = "Linear System: A\n$a\n\nB:\n$b\n\nC:\n$c\n\nD:\n$d\n"
}
