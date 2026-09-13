package alonlib.units

import alonlib.robotPrintError
import kotlin.math.absoluteValue

/** Represents a planar angle.
 *
 * Can be created from or converted to any of the following units:
 * - Radians
 * - Degrees
 * - Rotations
 * - Gradians
 */
class Angle(angle: Number, angleUnit: Unit) : Comparable<Angle> {

	private var radians = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("Angle is NaN.")
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("Angle is infinite.")
				0.0
			} else value
		}

	val asRadians get() = radians
	val asDegrees get() = this.inUnit(Unit.Degrees)
	val asRotations get() = this.inUnit(Unit.Rotations)
	val asGradians get() = this.inUnit(Unit.Gradians)

	init {
		radians = when (angleUnit) {
			Unit.Radians   -> angle.toDouble()
			Unit.Degrees   -> degToRad(angle)
			Unit.Rotations -> angle.toDouble() * (Math.PI * 2.0)
			Unit.Gradians  -> angle.toDouble() * (Math.PI / 200.0)
		}
	}

	private fun inUnit(angleUnit: Unit) =
		when (angleUnit) {
			Unit.Radians   -> radians
			Unit.Degrees   -> radToDeg(radians)
			Unit.Rotations -> radians / (Math.PI * 2.0)
			Unit.Gradians  -> radians / (Math.PI / 200.0)
		}

	val absoluteValue get() = fromRadians(radians.absoluteValue)

	override fun toString() = "Radians($radians)"
	override fun compareTo(other: Angle) = radians.compareTo(other.radians)

	operator fun plus(other: Angle) = fromRadians(radians + other.radians)
	operator fun minus(other: Angle) = fromRadians(radians - other.radians)
	operator fun times(other: Angle) = fromRadians(radians * other.radians)
	operator fun times(scalar: Number) = fromRadians(radians * scalar.toDouble())
	operator fun div(other: Angle) = fromRadians(radians / other.radians)
	operator fun div(scalar: Number) = fromRadians(radians / scalar.toDouble())
	operator fun unaryMinus() = fromRadians(-radians)

	fun coerceIn(min: Angle, max: Angle) = fromRadians(radians.coerceIn(min.radians, max.radians))

	enum class Unit {
		Radians,
		Degrees,
		Rotations,
		Gradians,
	}

	companion object {

		fun fromRadians(radians: Number) = Angle(radians, Unit.Radians)
		fun fromDegrees(degrees: Number) = Angle(degrees, Unit.Degrees)
		fun fromRotations(rotations: Number) = Angle(rotations, Unit.Rotations)
		fun fromGradians(gradians: Number) = Angle(gradians, Unit.Gradians)
	}
}
