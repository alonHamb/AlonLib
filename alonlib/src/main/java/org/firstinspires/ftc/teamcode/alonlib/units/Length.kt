package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a length.
 *
 * Can be created from or converted to any of the following units:
 * - Meters
 * - Centimeters
 * - Millimeters
 * - Feet
 * - Inches
 */
class Length(length: Number, lengthUnit: Unit) : Comparable<Length> {

	private var meters = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("Length is NaN.")
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("Length is infinite.")
				0.0
			} else value
		}

	val asMeters get() = meters
	val asCentimeters get() = this.inUnit(Unit.Centimeters)
	val asMillimeters get() = this.inUnit(Unit.Millimeters)
	val asFeet get() = this.inUnit(Unit.Feet)
	val asInches get() = this.inUnit(Unit.Inches)

	init {
		meters = when (lengthUnit) {
			Unit.Meters      -> length.toDouble()
			Unit.Centimeters -> length.toDouble() / 100
			Unit.Millimeters -> length.toDouble() / 1000
			Unit.Feet        -> feetToMeters(length)
			Unit.Inches      -> inchesToMeters(length)
		}
	}

	private fun inUnit(lengthUnit: Unit) =
		when (lengthUnit) {
			Unit.Meters      -> meters
			Unit.Centimeters -> meters * 100.0
			Unit.Millimeters -> meters * 1000.0
			Unit.Feet        -> metersToFeet(meters)
			Unit.Inches      -> metersToInches(meters)
		}

	override fun toString() = "Meters($meters)"
	override fun compareTo(other: Length) = meters.compareTo(other.meters)

	operator fun plus(other: Length) = fromMeters(meters + other.meters)
	operator fun minus(other: Length) = fromMeters(meters - other.meters)
	operator fun times(other: Length) = fromMeters(meters * other.meters)
	operator fun times(other: Number) = fromMeters(meters * other.toDouble())
	operator fun div(other: Length) = fromMeters(meters / other.meters)
	operator fun div(other: Number) = fromMeters(meters / other.toDouble())

	enum class Unit {
		Meters,
		Centimeters,
		Millimeters,
		Feet,
		Inches,
	}

	companion object {

		fun fromMeters(meters: Number) = Length(meters, Unit.Meters)
		fun fromCentimeters(centimeters: Number) = Length(centimeters, Unit.Centimeters)
		fun fromMillimeters(millimeters: Number) = Length(millimeters, Unit.Millimeters)
		fun fromFeet(feet: Number) = Length(feet, Unit.Feet)
		fun fromInches(inches: Number) = Length(inches, Unit.Inches)
	}
}
