package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Represents a duration.
 *
 * Can be created from or converted to any of the following units:
 * - Seconds
 * - Milliseconds
 * - Microseconds
 * - Nanoseconds
 * - Minutes
 * - Hours
 */
class Time(time: Number, timeUnit: Unit) : Comparable<Time> {
    private var seconds = 0.0
        set(value) {
            field = if (value.isNaN()) {
                robotPrintError("Time is NaN.")
                0.0
            } else if (value.isInfinite()) {
                robotPrintError("Time is infinite.")
                0.0
            } else value
        }

    val asSeconds get() = seconds
    val asMilliseconds get() = this.inUnit(Unit.Milliseconds)
    val asMicroseconds get() = this.inUnit(Unit.Microseconds)
    val asNanoseconds get() = this.inUnit(Unit.Nanoseconds)
    val asMinutes get() = this.inUnit(Unit.Minutes)
    val asHours get() = this.inUnit(Unit.Hours)

    init {
        seconds = when (timeUnit) {
            Unit.Seconds      -> time.toDouble()
            Unit.Milliseconds -> time.toDouble() / 1_000.0
            Unit.Microseconds -> time.toDouble() / 1_000_000.0
            Unit.Nanoseconds  -> time.toDouble() / 1_000_000_000.0
            Unit.Minutes      -> time.toDouble() * 60.0
            Unit.Hours        -> time.toDouble() * 3_600.0
        }
    }

    private fun inUnit(timeUnit: Unit) =
        when (timeUnit) {
            Unit.Seconds      -> seconds
            Unit.Milliseconds -> seconds * 1_000.0
            Unit.Microseconds -> seconds * 1_000_000.0
            Unit.Nanoseconds  -> seconds * 1_000_000_000.0
            Unit.Minutes      -> seconds / 60.0
            Unit.Hours        -> seconds / 3_600.0
        }

    override fun toString() = "Seconds($seconds)"
    override fun compareTo(other: Time) = seconds.compareTo(other.seconds)

    operator fun plus(other: Time) = fromSeconds(seconds + other.seconds)
    operator fun minus(other: Time) = fromSeconds(seconds - other.seconds)
    operator fun times(scalar: Double) = fromSeconds(seconds * scalar)
    operator fun div(scalar: Double) = fromSeconds(seconds / scalar)
    operator fun unaryMinus() = fromSeconds(-seconds)

    enum class Unit {
        Seconds,
        Milliseconds,
        Microseconds,
        Nanoseconds,
        Minutes,
        Hours,
    }

    companion object {
        fun fromSeconds(seconds: Number) = Time(seconds, Unit.Seconds)
        fun fromMilliseconds(milliseconds: Number) = Time(milliseconds, Unit.Milliseconds)
        fun fromMicroseconds(microseconds: Number) = Time(microseconds, Unit.Microseconds)
        fun fromNanoseconds(nanoseconds: Number) = Time(nanoseconds, Unit.Nanoseconds)
        fun fromMinutes(minutes: Number) = Time(minutes, Unit.Minutes)
        fun fromHours(hours: Number) = Time(hours, Unit.Hours)
    }
}
