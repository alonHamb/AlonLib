package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a percentage/ratio, e.g. motor percent output.
 *
 * Not clamped to any range by this class itself -- callers (like [HaMotor])
 * decide what range is valid for their use.
 *
 * Can be created from or converted to any of the following units:
 * - Fraction (0.0..1.0 == 0%..100%)
 * - Percent (0..100)
 * - Permille (0..1000, i.e. parts per thousand)
 * - BasisPoints (0..10000, i.e. parts per ten-thousand)
 */
class Percentage(percentage: Number, percentageUnit: Unit) : Comparable<Percentage> {

	private var fraction : Double = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("Percentage is NaN.")
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("Percentage is infinite.")
				0.0
			} else value
		}

	val asFraction get() = fraction
	val asPercent get() = this.inUnit(Unit.Percent)
	val asPermille get() = this.inUnit(Unit.Permille)
	val asBasisPoints get() = this.inUnit(Unit.BasisPoints)

	init {
		fraction = when (percentageUnit) {
			Unit.Fraction    -> percentage.toDouble()
			Unit.Percent     -> percentage.toDouble() / 100.0
			Unit.Permille    -> percentage.toDouble() / 1_000.0
			Unit.BasisPoints -> percentage.toDouble() / 10_000.0
		}
	}

	private fun inUnit(percentageUnit: Unit) =
		when (percentageUnit) {
			Unit.Fraction    -> fraction
			Unit.Percent     -> fraction * 100.0
			Unit.Permille    -> fraction * 1_000.0
			Unit.BasisPoints -> fraction * 10_000.0
		}

	override fun toString() = "Percentage($asPercent%)"
	override fun compareTo(other: Percentage) = fraction.compareTo(other.fraction)

	operator fun plus(other: Percentage) = fromFraction(fraction + other.fraction)
	operator fun minus(other: Percentage) = fromFraction(fraction - other.fraction)
	operator fun times(other: Percentage) = fromFraction(fraction * other.fraction)
	operator fun times(scalar: Number) = fromFraction(fraction * scalar.toDouble())
	operator fun div(other: Percentage) = fromFraction(fraction / other.fraction)
	operator fun div(scalar: Number = fromFraction(fraction / scalar.toDouble())
	operator fun unaryMinus() = fromFraction(-fraction)

	fun coerceIn(min: Percentage, max: Percentage) = fromFraction(fraction.coerceIn(min.fraction, max.fraction))

	enum class Unit {
		Fraction,
		Percent,
		Permille,
		BasisPoints,
	}

	companion object {

		fun fromFraction(fraction: Number) = Percentage(fraction, Unit.Fraction)
		fun fromPercent(percent: Number) = Percentage(percent, Unit.Percent)
		fun fromPermille(permille: Number) = Percentage(permille, Unit.Permille)
		fun fromBasisPoints(basisPoints: Number) = Percentage(basisPoints, Unit.BasisPoints)
	}
}
