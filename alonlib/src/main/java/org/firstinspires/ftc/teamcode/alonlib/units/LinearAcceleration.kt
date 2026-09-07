package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a linear (straight-line) acceleration.
 *
 * Can be created from or converted to any of the following units:
 * - Meters per second squared
 * - Feet per second squared
 * - Inches per second squared
 * - Standard gravities (g)
 */
class LinearAcceleration(acceleration: Number, accelerationUnit: Unit) : Comparable<LinearAcceleration> {

	private var metersPerSecondSquared = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("LinearAcceleration is NaN.")
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("LinearAcceleration is infinite.")
				0.0
			} else value
		}

	val asMetersPerSecondSquared get() = metersPerSecondSquared
	val asFeetPerSecondSquared get() = this.inUnit(Unit.FeetPerSecondSquared)
	val asInchesPerSecondSquared get() = this.inUnit(Unit.InchesPerSecondSquared)
	val asGs get() = this.inUnit(Unit.Gs)

	init {
		metersPerSecondSquared = when (accelerationUnit) {
			Unit.MetersPerSecondSquared -> acceleration.toDouble()
			Unit.FeetPerSecondSquared   -> feetToMeters(acceleration)
			Unit.InchesPerSecondSquared -> inchesToMeters(acceleration)
			Unit.Gs                     -> acceleration.toDouble() * STANDARD_GRAVITY
		}
	}

	private fun inUnit(accelerationUnit: Unit) =
		when (accelerationUnit) {
			Unit.MetersPerSecondSquared -> metersPerSecondSquared
			Unit.FeetPerSecondSquared   -> metersToFeet(metersPerSecondSquared)
			Unit.InchesPerSecondSquared -> metersToInches(metersPerSecondSquared)
			Unit.Gs                     -> metersPerSecondSquared / STANDARD_GRAVITY
		}

	override fun toString() = "MetersPerSecondSquared($metersPerSecondSquared)"
	override fun compareTo(other: LinearAcceleration) = metersPerSecondSquared.compareTo(other.metersPerSecondSquared)

	operator fun plus(other: LinearAcceleration) = fromMetersPerSecondSquared(metersPerSecondSquared + other.metersPerSecondSquared)
	operator fun minus(other: LinearAcceleration) = fromMetersPerSecondSquared(metersPerSecondSquared - other.metersPerSecondSquared)
	operator fun times(other: LinearAcceleration) = fromMetersPerSecondSquared(metersPerSecondSquared * other.metersPerSecondSquared)
	operator fun times(scalar: Double) = fromMetersPerSecondSquared(metersPerSecondSquared * scalar)
	operator fun div(other: LinearAcceleration) = fromMetersPerSecondSquared(metersPerSecondSquared / other.metersPerSecondSquared)
	operator fun div(scalar: Double) = fromMetersPerSecondSquared(metersPerSecondSquared / scalar)
	operator fun unaryMinus() = fromMetersPerSecondSquared(-metersPerSecondSquared)

	enum class Unit {
		MetersPerSecondSquared,
		FeetPerSecondSquared,
		InchesPerSecondSquared,
		Gs,
	}

	companion object {

		/** Standard gravity, in meters per second squared. */
		const val STANDARD_GRAVITY = 9.80665

		fun fromMetersPerSecondSquared(value: Number) = LinearAcceleration(value, Unit.MetersPerSecondSquared)
		fun fromFeetPerSecondSquared(value: Number) = LinearAcceleration(value, Unit.FeetPerSecondSquared)
		fun fromInchesPerSecondSquared(value: Number) = LinearAcceleration(value, Unit.InchesPerSecondSquared)
		fun fromGs(value: Number) = LinearAcceleration(value, Unit.Gs)
	}
}
