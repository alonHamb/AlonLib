package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.system.Discretization
import org.firstinspires.ftc.teamcode.alonlib.math.system.LinearSystem
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix

/**
 * A plant-inversion model-based feedforward: `u_ff = B⁺(r_{k+1} - A*r_k)`, where `B⁺` is the
 * pseudoinverse of B.
 */
class LinearPlantInversionFeedforward {

    private val a: Matrix
    private val b: Matrix

    var r: Matrix
        private set

    var uff: Matrix
        private set

    constructor(plant: LinearSystem, dtSeconds: Double) : this(plant.a, plant.b, dtSeconds)

    constructor(a: Matrix, b: Matrix, dtSeconds: Double) {
        val (discA, discB) = Discretization.discretizeAB(a, b, dtSeconds)
        this.a = discA
        this.b = discB

        r = Matrix(b.rows, 1)
        uff = Matrix(b.cols, 1)
    }

    /** Resets with a specified initial state [initialState]. */
    fun reset(initialState: Matrix) {
        r = initialState
        uff.fill(0.0)
    }

    /** Resets with a zero initial state. */
    fun reset() {
        r.fill(0.0)
        uff.fill(0.0)
    }

    /** The feedforward for tracking [nextR], continuing from the internally stored current reference. */
    fun calculate(nextR: Matrix) = calculate(r, nextR)

    /** The feedforward for going from reference [r] (at timestep k) to [nextR] (at timestep k+1). */
    fun calculate(r: Matrix, nextR: Matrix): Matrix {
        // rₖ₊₁ = Arₖ + Buₖ  =>  uₖ = B⁺(rₖ₊₁ − Arₖ)
        uff = b.solve(nextR - a * r)
        this.r = nextR
        return uff
    }
}
