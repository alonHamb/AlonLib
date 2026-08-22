package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a mass.
 *
 * Can be created from or converted to any of the following units:
 * - Kilograms
 * - Grams
 * - Pounds
 * - Ounces
 */
class Mass(mass: Number, massUnit: Unit) : Comparable<Mass> {
    private var kilograms = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("Mass is NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("Mass is infinite.")
                0.0
            } else value
        }

    val asKilograms get() = kilograms
    val asGrams get() = this.inUnit(Unit.Grams)
    val asPounds get() = this.inUnit(Unit.Pounds)
    val asOunces get() = this.inUnit(Unit.Ounces)

    init {
        kilograms = when (massUnit) {
            Unit.Kilograms -> mass.toDouble()
            Unit.Grams     -> mass.toDouble() / 1_000.0
            Unit.Pounds    -> mass.toDouble() * KILOGRAMS_PER_POUND
            Unit.Ounces    -> mass.toDouble() * KILOGRAMS_PER_POUND / 16.0
        }
    }

    private fun inUnit(massUnit: Unit) =
        when (massUnit) {
            Unit.Kilograms -> kilograms
            Unit.Grams     -> kilograms * 1_000.0
            Unit.Pounds    -> kilograms / KILOGRAMS_PER_POUND
            Unit.Ounces    -> kilograms / KILOGRAMS_PER_POUND * 16.0
        }

    override fun toString() = "Kilograms($kilograms)"
    override fun compareTo(other: Mass) = kilograms.compareTo(other.kilograms)

    operator fun plus(other: Mass) = fromKilograms(kilograms + other.kilograms)
    operator fun minus(other: Mass) = fromKilograms(kilograms - other.kilograms)
    operator fun times(scalar: Double) = fromKilograms(kilograms * scalar)
    operator fun div(scalar: Double) = fromKilograms(kilograms / scalar)
    operator fun unaryMinus() = fromKilograms(-kilograms)

    enum class Unit {
        Kilograms,
        Grams,
        Pounds,
        Ounces,
    }

    companion object {
        const val KILOGRAMS_PER_POUND = 0.45359237

        fun fromKilograms(value: Number) = Mass(value, Unit.Kilograms)
        fun fromGrams(value: Number) = Mass(value, Unit.Grams)
        fun fromPounds(value: Number) = Mass(value, Unit.Pounds)
        fun fromOunces(value: Number) = Mass(value, Unit.Ounces)
    }
}
