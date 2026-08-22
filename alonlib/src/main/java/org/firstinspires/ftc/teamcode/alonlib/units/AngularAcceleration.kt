package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents an angular acceleration.
 *
 * Can be created from or converted to any of the following units:
 * - Radians per second squared
 * - Degrees per second squared
 * - Rotations per second squared
 * - Rpm per second (common motor spec unit)
 */
class AngularAcceleration(acceleration: Number, accelerationUnit: Unit) : Comparable<AngularAcceleration> {
    private var radiansPerSecondSquared = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("AngularAcceleration is NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("AngularAcceleration is infinite.")
                0.0
            } else value
        }

    val asRadiansPerSecondSquared get() = radiansPerSecondSquared
    val asDegreesPerSecondSquared get() = this.inUnit(Unit.DegreesPerSecondSquared)
    val asRotationsPerSecondSquared get() = this.inUnit(Unit.RotationsPerSecondSquared)
    val asRpmPerSecond get() = this.inUnit(Unit.RpmPerSecond)

    init {
        radiansPerSecondSquared = when (accelerationUnit) {
            Unit.RadiansPerSecondSquared   -> acceleration.toDouble()
            Unit.DegreesPerSecondSquared   -> degToRad(acceleration)
            Unit.RotationsPerSecondSquared -> acceleration.toDouble() * (Math.PI * 2.0)
            Unit.RpmPerSecond              -> rpmToRadPs(acceleration)
        }
    }

    private fun inUnit(accelerationUnit: Unit) =
        when (accelerationUnit) {
            Unit.RadiansPerSecondSquared   -> radiansPerSecondSquared
            Unit.DegreesPerSecondSquared   -> radToDeg(radiansPerSecondSquared)
            Unit.RotationsPerSecondSquared -> radiansPerSecondSquared / (Math.PI * 2.0)
            Unit.RpmPerSecond              -> radPsToRpm(radiansPerSecondSquared)
        }

    override fun toString() = "RadiansPerSecondSquared($radiansPerSecondSquared)"
    override fun compareTo(other: AngularAcceleration) = radiansPerSecondSquared.compareTo(other.radiansPerSecondSquared)

    operator fun plus(other: AngularAcceleration) = fromRadiansPerSecondSquared(radiansPerSecondSquared + other.radiansPerSecondSquared)
    operator fun minus(other: AngularAcceleration) = fromRadiansPerSecondSquared(radiansPerSecondSquared - other.radiansPerSecondSquared)
    operator fun times(scalar: Double) = fromRadiansPerSecondSquared(radiansPerSecondSquared * scalar)
    operator fun div(scalar: Double) = fromRadiansPerSecondSquared(radiansPerSecondSquared / scalar)
    operator fun unaryMinus() = fromRadiansPerSecondSquared(-radiansPerSecondSquared)

    enum class Unit {
        RadiansPerSecondSquared,
        DegreesPerSecondSquared,
        RotationsPerSecondSquared,
        RpmPerSecond,
    }

    companion object {
        fun fromRadiansPerSecondSquared(value: Number) = AngularAcceleration(value, Unit.RadiansPerSecondSquared)
        fun fromDegreesPerSecondSquared(value: Number) = AngularAcceleration(value, Unit.DegreesPerSecondSquared)
        fun fromRotationsPerSecondSquared(value: Number) = AngularAcceleration(value, Unit.RotationsPerSecondSquared)
        fun fromRpmPerSecond(value: Number) = AngularAcceleration(value, Unit.RpmPerSecond)
    }
}
