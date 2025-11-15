package org.firstinspires.ftc.teamcode.alonlib.units

import com.seattlesolvers.solverslib.geometry.Rotation2d
import kotlin.math.abs

// --- Length ---

inline val Number.meters get() = Length.fromMeters(this.toDouble())
inline val Number.centimeters get() = Length.fromCentimeters(this.toDouble())
inline val Number.millimeters get() = Length.fromMillimeters(this.toDouble())
inline val Number.feet get() = Length.fromFeet(this.toDouble())
inline val Number.inches get() = Length.fromInches(this.toDouble())

// --- Angular Velocity ---

inline val Number.rpm get() = AngularVelocity.fromRpm(this.toDouble())
inline val Number.rps get() = AngularVelocity.fromRps(this.toDouble())
inline val Number.radPs get() = AngularVelocity.fromRadPs(this.toDouble())
inline val Number.degPs get() = AngularVelocity.fromDegPs(this.toDouble())

// --- Rotation2d ---

inline val Number.degrees: Rotation2d get() = Rotation2d.fromDegrees(this.toDouble())
inline val Number.radians: Rotation2d get() = Rotation2d(this.toDouble())
inline val Number.rotations: Rotation2d get() = Rotation2d.fromDegrees(this.toDouble() * 360.0)


inline val Rotation2d.absoluteValue: Rotation2d get() = Rotation2d.fromDegrees(abs(this.degrees))
inline val Rotation2d.rotations: Number get() = (this.degrees / 360.0)

infix fun Rotation2d.plus(other: Rotation2d) = (this.degrees + other.degrees).degrees
infix fun Rotation2d.minus(other: Rotation2d) = (this.degrees - other.degrees).degrees

