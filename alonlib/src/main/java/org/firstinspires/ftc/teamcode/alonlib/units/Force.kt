package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a force.
 *
 * Can be created from or converted to any of the following units:
 * - Newtons
 * - Pounds-force
 * - Kilograms-force
 * - Dynes
 */
class Force(force: Number, forceUnit: Unit) : Comparable<Force> {

	private var newtons = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("Force is NaN.")
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("Force is infinite.")
				0.0
			} else value
		}

	val asNewtons get() = newtons
	val asPoundsForce get() = this.inUnit(Unit.PoundsForce)
	val asKilogramsForce get() = this.inUnit(Unit.KilogramsForce)
	val asDynes get() = this.inUnit(Unit.Dynes)

	init {
		newtons = when (forceUnit) {
			Unit.Newtons        -> force.toDouble()
			Unit.PoundsForce    -> force.toDouble() * Mass.KILOGRAMS_PER_POUND * LinearAcceleration.STANDARD_GRAVITY
			Unit.KilogramsForce -> force.toDouble() * LinearAcceleration.STANDARD_GRAVITY
			Unit.Dynes          -> force.toDouble() / 100_000.0
		}
	}

	private fun inUnit(forceUnit: Unit) =
		when (forceUnit) {
			Unit.Newtons        -> newtons
			Unit.PoundsForce    -> newtons / (Mass.KILOGRAMS_PER_POUND * LinearAcceleration.STANDARD_GRAVITY)
			Unit.KilogramsForce -> newtons / LinearAcceleration.STANDARD_GRAVITY
			Unit.Dynes          -> newtons * 100_000.0
		}

	override fun toString() = "Newtons($newtons)"
	override fun compareTo(other: Force) = newtons.compareTo(other.newtons)

	operator fun plus(other: Force) = fromNewtons(newtons + other.newtons)
	operator fun minus(other: Force) = fromNewtons(newtons - other.newtons)
	operator fun times(other: Force) = fromNewtons(newtons * other.newtons)
	operator fun times(scalar: Double) = fromNewtons(newtons * scalar)
	operator fun div(other: Force) = fromNewtons(other.newtons)
	operator fun div(scalar: Double) = fromNewtons(newtons / scalar)
	operator fun unaryMinus() = fromNewtons(-newtons)

	enum class Unit {
		Newtons,
		PoundsForce,
		KilogramsForce,
		Dynes,
	}

	companion object {

		fun fromNewtons(value: Number) = Force(value, Unit.Newtons)
		fun fromPoundsForce(value: Number) = Force(value, Unit.PoundsForce)
		fun fromKilogramsForce(value: Number) = Force(value, Unit.KilogramsForce)
		fun fromDynes(value: Number) = Force(value, Unit.Dynes)
	}
}
