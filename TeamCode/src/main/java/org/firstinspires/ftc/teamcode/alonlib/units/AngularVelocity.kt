package com.hamosad1657.lib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.Length
import org.firstinspires.ftc.teamcode.alonlib.units.degPsToRpm
import org.firstinspires.ftc.teamcode.alonlib.units.mpsToRpm
import org.firstinspires.ftc.teamcode.alonlib.units.radPsToRpm
import org.firstinspires.ftc.teamcode.alonlib.units.rpmToDegPs
import org.firstinspires.ftc.teamcode.alonlib.units.rpmToMps
import org.firstinspires.ftc.teamcode.alonlib.units.rpmToRadPs
import org.firstinspires.ftc.teamcode.alonlib.units.rpmToRps
import org.firstinspires.ftc.teamcode.alonlib.units.rpsToRpm
import kotlin.math.absoluteValue
import com.hamosad1657.lib.units.AngularVelocity.Unit as AngularVelocityUnit

/** Represents an angular velocity.
 *
 * Can be created from or converted to any of the following units:
 * - Rotations per Minute (Rpm)
 * - Rotations per Second (Rps)
 * - Radians per Second (RadPs)
 * - Degrees per Second (DegPs)
 *
 * Can also be converted to:
 * - Meters per Second (Mps)
 * - Falcon's Integrated Encoder Ticks per 100ms
 */
class AngularVelocity
private constructor(velocity: Double, velocityUnit: AngularVelocityUnit) :
    Comparable<AngularVelocity> {
    private var rpm = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("AngularVelocity cannot be NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("AngularVelocity cannot be infinite.")
                0.0
            } else value
        }

    val asRpm get() = rpm
    val asRps get() = this.inUnit(AngularVelocityUnit.Rps)
    val asRadPs get() = this.inUnit(AngularVelocityUnit.RadPs)
    val asDegPs get() = this.inUnit(AngularVelocityUnit.DegPs)

    fun asMps(wheelRadius: Length) = rpmToMps(rpm, wheelRadius)

    init {
        rpm = when (velocityUnit) {
            AngularVelocityUnit.Rpm -> velocity
            AngularVelocityUnit.Rps -> rpsToRpm(velocity)
            AngularVelocityUnit.RadPs -> radPsToRpm(velocity)
            AngularVelocityUnit.DegPs -> degPsToRpm(velocity)
        }
    }

    private fun inUnit(velocityUnit: AngularVelocityUnit) =
        when (velocityUnit) {
            AngularVelocityUnit.Rpm -> rpm
            AngularVelocityUnit.Rps -> rpmToRps(rpm)
            AngularVelocityUnit.RadPs -> rpmToRadPs(rpm)
            AngularVelocityUnit.DegPs -> rpmToDegPs(rpm)
        }

    val absoluteValue get() = fromRpm(rpm.absoluteValue)

    override fun toString() = "RPM($rpm)"
    override fun compareTo(other: AngularVelocity): Int = (rpm - other.rpm).toInt()

    operator fun plus(other: AngularVelocity) = fromRpm(rpm + other.rpm)
    operator fun minus(other: AngularVelocity) = fromRpm(rpm - other.rpm)
    operator fun times(ratio: Double) = fromRpm(rpm * ratio)
    operator fun div(ratio: Double) = fromRpm(rpm / ratio)

    enum class Unit {
        Rpm,
        Rps,
        RadPs,
        DegPs,
    }

    companion object {
        fun fromRpm(rpm: Double) = AngularVelocity(rpm, AngularVelocityUnit.Rpm)
        fun fromRps(rps: Double) = AngularVelocity(rps, AngularVelocityUnit.Rps)
        fun fromRadPs(radPs: Double) = AngularVelocity(radPs, AngularVelocityUnit.RadPs)
        fun fromDegPs(degPs: Double) = AngularVelocity(degPs, AngularVelocityUnit.DegPs)
        fun fromMps(mps: Double, wheelRadius: Length) = fromRpm(mpsToRpm(mps, wheelRadius))
    }
}
