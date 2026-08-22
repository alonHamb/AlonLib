package org.firstinspires.ftc.teamcode.alonlib.math.control

import kotlin.math.abs

/**
 * The simplest possible controller: outputs 1 if [measurement] < [setPoint], else 0. Extremely
 * aggressive, works well for velocity control of high-inertia mechanisms (flywheels), poorly for
 * almost anything else.
 *
 * This is *asymmetric* -- it never commands negative output, so it can't slow an overspeeding
 * mechanism down, only stop pushing it. Make sure motor controllers are set to coast (not brake)
 * before driving them with this.
 */
class BangBangController(var tolerance: Double = Double.POSITIVE_INFINITY) {

    var setPoint = 0.0
        private set

    var measurement = 0.0
        private set

    val error get() = setPoint - measurement

    fun atSetPoint() = abs(error) < tolerance

    /** @returns the calculated output (0 or 1) for [measurement] against [setPoint]. */
    fun calculate(measurement: Double, setPoint: Double): Double {
        this.measurement = measurement
        this.setPoint = setPoint
        return if (measurement < setPoint) 1.0 else 0.0
    }

    /** @returns the calculated output (0 or 1) for [measurement] against the last-set [setPoint]. */
    fun calculate(measurement: Double) = calculate(measurement, setPoint)
}
