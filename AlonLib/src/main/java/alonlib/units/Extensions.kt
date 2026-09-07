package alonlib.units

import alonlib.math.geometry.Pose2d
import alonlib.math.geometry.Rotation2d
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt

// --- Length ---

inline val Number.meters get() = Length.Companion.fromMeters(this.toDouble())
inline val Number.centimeters get() = Length.Companion.fromCentimeters(this.toDouble())
inline val Number.millimeters get() = Length.Companion.fromMillimeters(this.toDouble())
inline val Number.feet get() = Length.Companion.fromFeet(this.toDouble())
inline val Number.inches get() = Length.Companion.fromInches(this.toDouble())

// --- Angular Velocity ---

inline val Number.rpm get() = AngularVelocity.fromRpm(this.toDouble())
inline val Number.rps get() = AngularVelocity.fromRps(this.toDouble())
inline val Number.radPs get() = AngularVelocity.fromRadPs(this.toDouble())
inline val Number.degPs get() = AngularVelocity.fromDegPs(this.toDouble())

// --- Linear Velocity ---

inline val Number.metersPerSecond get() = LinearVelocity.Companion.fromMetersPerSecond(this)
inline val Number.feetPerSecond get() = LinearVelocity.Companion.fromFeetPerSecond(this)
inline val Number.inchesPerSecond get() = LinearVelocity.Companion.fromInchesPerSecond(this)
inline val Number.centimetersPerSecond get() = LinearVelocity.Companion.fromCentimetersPerSecond(this)
inline val Number.millimetersPerSecond get() = LinearVelocity.Companion.fromMillimetersPerSecond(this)
inline val Number.kilometersPerHour get() = LinearVelocity.Companion.fromKilometersPerHour(this)
inline val Number.milesPerHour get() = LinearVelocity.Companion.fromMilesPerHour(this)

// --- Linear Acceleration ---

inline val Number.metersPerSecondSquared get() = LinearAcceleration.Companion.fromMetersPerSecondSquared(this)
inline val Number.feetPerSecondSquared get() = LinearAcceleration.Companion.fromFeetPerSecondSquared(this)
inline val Number.inchesPerSecondSquared get() = LinearAcceleration.Companion.fromInchesPerSecondSquared(this)
inline val Number.gs get() = LinearAcceleration.Companion.fromGs(this)

// --- Angular Acceleration ---

inline val Number.radPs2 get() = AngularAcceleration.fromRadiansPerSecondSquared(this)
inline val Number.degPs2 get() = AngularAcceleration.fromDegreesPerSecondSquared(this)
inline val Number.rps2 get() = AngularAcceleration.fromRotationsPerSecondSquared(this)
inline val Number.rpmPerSecond get() = AngularAcceleration.fromRpmPerSecond(this)

// --- Mass ---

inline val Number.kilograms get() = Mass.Companion.fromKilograms(this)
inline val Number.grams get() = Mass.Companion.fromGrams(this)
inline val Number.pounds get() = Mass.Companion.fromPounds(this)
inline val Number.ounces get() = Mass.Companion.fromOunces(this)

// --- Force ---

inline val Number.newtons get() = Force.Companion.fromNewtons(this)
inline val Number.poundsForce get() = Force.Companion.fromPoundsForce(this)
inline val Number.kilogramsForce get() = Force.Companion.fromKilogramsForce(this)
inline val Number.dynes get() = Force.Companion.fromDynes(this)

// --- Torque ---

inline val Number.newtonMeters get() = Torque.Companion.fromNewtonMeters(this)
inline val Number.poundFeet get() = Torque.Companion.fromPoundFeet(this)
inline val Number.poundInches get() = Torque.Companion.fromPoundInches(this)
inline val Number.ounceInches get() = Torque.Companion.fromOunceInches(this)
inline val Number.kilogramCentimeters get() = Torque.Companion.fromKilogramCentimeters(this)

// --- Voltage ---

inline val Number.volts get() = Voltage.Companion.fromVolts(this)
inline val Number.millivolts get() = Voltage.Companion.fromMillivolts(this)
inline val Number.microvolts get() = Voltage.Companion.fromMicrovolts(this)
inline val Number.kilovolts get() = Voltage.Companion.fromKilovolts(this)

// --- Current ---

inline val Number.amps get() = Current.fromAmps(this)
inline val Number.milliamps get() = Current.fromMilliamps(this)
inline val Number.microamps get() = Current.fromMicroamps(this)
inline val Number.kiloamps get() = Current.fromKiloamps(this)

// --- Time ---

inline val Number.seconds get() = Time.Companion.fromSeconds(this)
inline val Number.milliseconds get() = Time.Companion.fromMilliseconds(this)
inline val Number.microseconds get() = Time.Companion.fromMicroseconds(this)
inline val Number.nanoseconds get() = Time.Companion.fromNanoseconds(this)
inline val Number.minutes get() = Time.Companion.fromMinutes(this)
inline val Number.hours get() = Time.Companion.fromHours(this)

// --- Percentage ---

inline val Number.fraction get() = Percentage.Companion.fromFraction(this)
inline val Number.percent get() = Percentage.Companion.fromPercent(this)
inline val Number.permille get() = Percentage.Companion.fromPermille(this)
inline val Number.basisPoints get() = Percentage.Companion.fromBasisPoints(this)

// --- Rotation2d ---

inline val Number.degrees: Rotation2d get() = Rotation2d.fromDegrees(this.toDouble())
inline val Number.radians: Rotation2d get() = Rotation2d(this.toDouble())
inline val Number.rotations: Rotation2d get() = Rotation2d.fromDegrees(this.toDouble() * 360.0)


inline val Rotation2d.absoluteValue: Rotation2d get() = Rotation2d.fromDegrees(abs(this.degrees))

/** [degrees] wrapped into `[0, 360)`, e.g. -170deg (how [Rotation2d] itself canonicalizes 190deg) reads back as 190deg here. */
inline val Rotation2d.normalizedDegrees: Double get() = ((this.degrees % 360.0) + 360.0) % 360.0

/** [radians] wrapped into `[0, 2*pi)`. */
inline val Rotation2d.normalizedRadians: Double get() = ((this.radians % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)

/** [rotations] wrapped into `[0, 1)`. */
inline val Rotation2d.normalizedRotations: Double get() = normalizedDegrees / 360.0


operator fun Rotation2d.times(ratio: Double) = (this.degrees * ratio).degrees
operator fun Rotation2d.div(ratio: Double) = (this.degrees / ratio).degrees
operator fun Rotation2d.rangeTo(that: Rotation2d) = (this.degrees.rangeTo(that.degrees))
operator fun Rotation2d.compareTo(other: Rotation2d) = (this.degrees.compareTo(other.degrees))
// --- Position ---

fun Pose2d.xDistanceTo(other: Pose2d) = other.x - this.x
fun Pose2d.yDistanceTO(other: Pose2d) = other.y - this.y
fun Pose2d.distanceTo(other: Pose2d) = sqrt(this.xDistanceTo(other).pow(2) + this.yDistanceTO(other).pow(2))
fun Pose2d.horizontalDistanceTo(other: Pose2d) = sqrt(this.xDistanceTo(other).pow(2) + this.yDistanceTO(other).pow(2))

// atan() returns radians, so this must go through .radians, not .degrees -- .degrees would treat
// the raw radian value as if it were already in degrees (e.g. atan(1) == pi/4 becoming "0.785deg"
// instead of 45deg).
fun Pose2d.horizontalAngleTo(other: Pose2d): Rotation2d = atan(this.yDistanceTO(other) / this.xDistanceTo(other)).radians
