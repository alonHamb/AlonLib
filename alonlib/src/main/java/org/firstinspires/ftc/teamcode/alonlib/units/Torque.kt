package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a torque.
 *
 * Can be created from or converted to any of the following units:
 * - Newton-meters
 * - Pound-feet
 * - Pound-inches
 * - Ounce-inches (common servo spec unit)
 * - Kilogram-centimeters (common servo spec unit)
 */
class Torque(torque: Number, torqueUnit: Unit) : Comparable<Torque> {
    private var newtonMeters = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("Torque is NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("Torque is infinite.")
                0.0
            } else value
        }

    val asNewtonMeters get() = newtonMeters
    val asPoundFeet get() = this.inUnit(Unit.PoundFeet)
    val asPoundInches get() = this.inUnit(Unit.PoundInches)
    val asOunceInches get() = this.inUnit(Unit.OunceInches)
    val asKilogramCentimeters get() = this.inUnit(Unit.KilogramCentimeters)

    init {
        newtonMeters = when (torqueUnit) {
            Unit.NewtonMeters        -> torque.toDouble()
            Unit.PoundFeet           -> torque.toDouble() * NEWTON_METERS_PER_POUND_FOOT
            Unit.PoundInches         -> torque.toDouble() * NEWTON_METERS_PER_POUND_FOOT / 12.0
            Unit.OunceInches         -> torque.toDouble() * NEWTON_METERS_PER_POUND_FOOT / 12.0 / 16.0
            Unit.KilogramCentimeters -> torque.toDouble() * LinearAcceleration.STANDARD_GRAVITY / 100.0
        }
    }

    private fun inUnit(torqueUnit: Unit) =
        when (torqueUnit) {
            Unit.NewtonMeters        -> newtonMeters
            Unit.PoundFeet           -> newtonMeters / NEWTON_METERS_PER_POUND_FOOT
            Unit.PoundInches         -> newtonMeters / NEWTON_METERS_PER_POUND_FOOT * 12.0
            Unit.OunceInches         -> newtonMeters / NEWTON_METERS_PER_POUND_FOOT * 12.0 * 16.0
            Unit.KilogramCentimeters -> newtonMeters / LinearAcceleration.STANDARD_GRAVITY * 100.0
        }

    override fun toString() = "NewtonMeters($newtonMeters)"
    override fun compareTo(other: Torque) = newtonMeters.compareTo(other.newtonMeters)

    operator fun plus(other: Torque) = fromNewtonMeters(newtonMeters + other.newtonMeters)
    operator fun minus(other: Torque) = fromNewtonMeters(newtonMeters - other.newtonMeters)
    operator fun times(scalar: Double) = fromNewtonMeters(newtonMeters * scalar)
    operator fun div(scalar: Double) = fromNewtonMeters(newtonMeters / scalar)
    operator fun unaryMinus() = fromNewtonMeters(-newtonMeters)

    enum class Unit {
        NewtonMeters,
        PoundFeet,
        PoundInches,
        OunceInches,
        KilogramCentimeters,
    }

    companion object {
        const val NEWTON_METERS_PER_POUND_FOOT = Mass.KILOGRAMS_PER_POUND * LinearAcceleration.STANDARD_GRAVITY * 0.3048

        fun fromNewtonMeters(value: Number) = Torque(value, Unit.NewtonMeters)
        fun fromPoundFeet(value: Number) = Torque(value, Unit.PoundFeet)
        fun fromPoundInches(value: Number) = Torque(value, Unit.PoundInches)
        fun fromOunceInches(value: Number) = Torque(value, Unit.OunceInches)
        fun fromKilogramCentimeters(value: Number) = Torque(value, Unit.KilogramCentimeters)
    }
}
