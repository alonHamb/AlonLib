package org.firstinspires.ftc.teamcode.alonlib.math.system

/**
 * 4th-order Runge-Kutta numerical integration. Upstream WPILib also has an adaptive
 * Dormand-Prince integrator (`rkdp`) and a time-varying `f(t, y)` overload of `rk4` -- neither is
 * exercised anywhere in this port (state estimation here only needs fixed-step `f(x, u)`/`f(x)`
 * integration), so both are left out rather than carried as untested dead code.
 */
object NumericalIntegration {

    /** Integrates `dx/dt = f(x)` for [dtSeconds], starting at [x]. */
    fun rk4(f: (Double) -> Double, x: Double, dtSeconds: Double): Double {
        val h = dtSeconds
        val k1 = f(x)
        val k2 = f(x + h * k1 * 0.5)
        val k3 = f(x + h * k2 * 0.5)
        val k4 = f(x + h * k3)
        return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    }

    /** Integrates `dx/dt = f(x, u)` for [dtSeconds], holding [u] constant over the step. */
    fun rk4(f: (Matrix, Matrix) -> Matrix, x: Matrix, u: Matrix, dtSeconds: Double): Matrix {
        val h = dtSeconds
        val k1 = f(x, u)
        val k2 = f(x + k1 * (h * 0.5), u)
        val k3 = f(x + k2 * (h * 0.5), u)
        val k4 = f(x + k3 * h, u)
        return x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0)
    }

    /** Integrates `dx/dt = f(x)` for [dtSeconds]. */
    fun rk4(f: (Matrix) -> Matrix, x: Matrix, dtSeconds: Double): Matrix {
        val h = dtSeconds
        val k1 = f(x)
        val k2 = f(x + k1 * (h * 0.5))
        val k3 = f(x + k2 * (h * 0.5))
        val k4 = f(x + k3 * h)
        return x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0)
    }
}
