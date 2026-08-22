package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a voltage.
 *
 * Can be created from or converted to any of the following units:
 * - Volts
 * - Millivolts
 * - Microvolts
 * - Kilovolts
 */
class Voltage(voltage: Number, voltageUnit: Unit) : Comparable<Voltage> {
    private var volts = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("Voltage is NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("Voltage is infinite.")
                0.0
            } else value
        }

    val asVolts get() = volts
    val asMillivolts get() = this.inUnit(Unit.Millivolts)
    val asMicrovolts get() = this.inUnit(Unit.Microvolts)
    val asKilovolts get() = this.inUnit(Unit.Kilovolts)

    init {
        volts = when (voltageUnit) {
            Unit.Volts      -> voltage.toDouble()
            Unit.Millivolts -> voltage.toDouble() / 1_000.0
            Unit.Microvolts -> voltage.toDouble() / 1_000_000.0
            Unit.Kilovolts  -> voltage.toDouble() * 1_000.0
        }
    }

    private fun inUnit(voltageUnit: Unit) =
        when (voltageUnit) {
            Unit.Volts      -> volts
            Unit.Millivolts -> volts * 1_000.0
            Unit.Microvolts -> volts * 1_000_000.0
            Unit.Kilovolts  -> volts / 1_000.0
        }

    override fun toString() = "Volts($volts)"
    override fun compareTo(other: Voltage) = volts.compareTo(other.volts)

    operator fun plus(other: Voltage) = fromVolts(volts + other.volts)
    operator fun minus(other: Voltage) = fromVolts(volts - other.volts)
    operator fun times(scalar: Double) = fromVolts(volts * scalar)
    operator fun div(scalar: Double) = fromVolts(volts / scalar)
    operator fun unaryMinus() = fromVolts(-volts)

    enum class Unit {
        Volts,
        Millivolts,
        Microvolts,
        Kilovolts,
    }

    companion object {
        fun fromVolts(volts: Number) = Voltage(volts, Unit.Volts)
        fun fromMillivolts(millivolts: Number) = Voltage(millivolts, Unit.Millivolts)
        fun fromMicrovolts(microvolts: Number) = Voltage(microvolts, Unit.Microvolts)
        fun fromKilovolts(kilovolts: Number) = Voltage(kilovolts, Unit.Kilovolts)
    }
}
