package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a linear (straight-line) velocity.
 *
 * Can be created from or converted to any of the following units:
 * - Meters per second
 * - Feet per second
 * - Inches per second
 * - Centimeters per second
 * - Millimeters per second
 * - Kilometers per hour
 * - Miles per hour
 */
class LinearVelocity(velocity: Number, velocityUnit: Unit) : Comparable<LinearVelocity> {

	private var metersPerSecond = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("LinearVelocity is NaN.")
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("LinearVelocity is infinite.")
				0.0
			} else value
		}

	val asMetersPerSecond get() = metersPerSecond
	val asFeetPerSecond get() = this.inUnit(Unit.FeetPerSecond)
	val asInchesPerSecond get() = this.inUnit(Unit.InchesPerSecond)
	val asCentimetersPerSecond get() = this.inUnit(Unit.CentimetersPerSecond)
	val asMillimetersPerSecond get() = this.inUnit(Unit.MillimetersPerSecond)
	val asKilometersPerHour get() = this.inUnit(Unit.KilometersPerHour)
	val asMilesPerHour get() = this.inUnit(Unit.MilesPerHour)

	init {
		metersPerSecond = when (velocityUnit) {
			Unit.MetersPerSecond      -> velocity.toDouble()
			Unit.FeetPerSecond        -> feetToMeters(velocity)
			Unit.InchesPerSecond      -> inchesToMeters(velocity)
			Unit.CentimetersPerSecond -> velocity.toDouble() / 100.0
			Unit.MillimetersPerSecond -> velocity.toDouble() / 1_000.0
			Unit.KilometersPerHour    -> velocity.toDouble() / 3.6
			Unit.MilesPerHour         -> velocity.toDouble() / mpsToMph(1.0)
		}
	}

	private fun inUnit(velocityUnit: Unit) =
		when (velocityUnit) {
			Unit.MetersPerSecond      -> metersPerSecond
			Unit.FeetPerSecond        -> metersToFeet(metersPerSecond)
			Unit.InchesPerSecond      -> metersToInches(metersPerSecond)
			Unit.CentimetersPerSecond -> metersPerSecond * 100.0
			Unit.MillimetersPerSecond -> metersPerSecond * 1_000.0
			Unit.KilometersPerHour    -> metersPerSecond * 3.6
			Unit.MilesPerHour         -> mpsToMph(metersPerSecond)
		}

	override fun toString() = "MetersPerSecond($metersPerSecond)"
	override fun compareTo(other: LinearVelocity) = metersPerSecond.compareTo(other.metersPerSecond)

	operator fun plus(other: LinearVelocity) = fromMetersPerSecond(metersPerSecond + other.metersPerSecond)
	operator fun minus(other: LinearVelocity) = fromMetersPerSecond(metersPerSecond - other.metersPerSecond)
	operator fun times(other: LinearVelocity) = fromMetersPerSecond(metersPerSecond * other.metersPerSecond)
	operator fun times(scalar: Double) = fromMetersPerSecond(metersPerSecond * scalar)
	operator fun div(other: LinearVelocity) = fromMetersPerSecond(metersPerSecond / other.metersPerSecond)
	operator fun div(scalar: Double) = fromMetersPerSecond(metersPerSecond / scalar)
	operator fun unaryMinus() = fromMetersPerSecond(-metersPerSecond)

	enum class Unit {
		MetersPerSecond,
		FeetPerSecond,
		InchesPerSecond,
		CentimetersPerSecond,
		MillimetersPerSecond,
		KilometersPerHour,
		MilesPerHour,
	}

	companion object {

		fun fromMetersPerSecond(value: Number) = LinearVelocity(value, Unit.MetersPerSecond)
		fun fromFeetPerSecond(value: Number) = LinearVelocity(value, Unit.FeetPerSecond)
		fun fromInchesPerSecond(value: Number) = LinearVelocity(value, Unit.InchesPerSecond)
		fun fromCentimetersPerSecond(value: Number) = LinearVelocity(value, Unit.CentimetersPerSecond)
		fun fromMillimetersPerSecond(value: Number) = LinearVelocity(value, Unit.MillimetersPerSecond)
		fun fromKilometersPerHour(value: Number) = LinearVelocity(value, Unit.KilometersPerHour)
		fun fromMilesPerHour(value: Number) = LinearVelocity(value, Unit.MilesPerHour)
	}
}
