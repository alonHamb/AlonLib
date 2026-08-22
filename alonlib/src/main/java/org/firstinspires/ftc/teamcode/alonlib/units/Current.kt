package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents an electrical current.
 *
 * Can be created from or converted to any of the following units:
 * - Amps
 * - Milliamps
 * - Microamps
 * - Kiloamps
 */
class Current(current: Number, currentUnit: Unit) : Comparable<Current> {
    private var amps = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("Current is NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("Current is infinite.")
                0.0
            } else value
        }

    val asAmps get() = amps
    val asMilliamps get() = this.inUnit(Unit.Milliamps)
    val asMicroamps get() = this.inUnit(Unit.Microamps)
    val asKiloamps get() = this.inUnit(Unit.Kiloamps)

    init {
        amps = when (currentUnit) {
            Unit.Amps      -> current.toDouble()
            Unit.Milliamps -> current.toDouble() / 1_000.0
            Unit.Microamps -> current.toDouble() / 1_000_000.0
            Unit.Kiloamps  -> current.toDouble() * 1_000.0
        }
    }

    private fun inUnit(currentUnit: Unit) =
        when (currentUnit) {
            Unit.Amps      -> amps
            Unit.Milliamps -> amps * 1_000.0
            Unit.Microamps -> amps * 1_000_000.0
            Unit.Kiloamps  -> amps / 1_000.0
        }

    override fun toString() = "Amps($amps)"
    override fun compareTo(other: Current) = amps.compareTo(other.amps)

    operator fun plus(other: Current) = fromAmps(amps + other.amps)
    operator fun minus(other: Current) = fromAmps(amps - other.amps)
    operator fun times(scalar: Double) = fromAmps(amps * scalar)
    operator fun div(scalar: Double) = fromAmps(amps / scalar)
    operator fun unaryMinus() = fromAmps(-amps)

    enum class Unit {
        Amps,
        Milliamps,
        Microamps,
        Kiloamps,
    }

    companion object {
        fun fromAmps(amps: Number) = Current(amps, Unit.Amps)
        fun fromMilliamps(milliamps: Number) = Current(milliamps, Unit.Milliamps)
        fun fromMicroamps(microamps: Number) = Current(microamps, Unit.Microamps)
        fun fromKiloamps(kiloamps: Number) = Current(kiloamps, Unit.Kiloamps)
    }
}
