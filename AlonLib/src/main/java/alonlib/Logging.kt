package alonlib

import alonlib.units.Time
import alonlib.units.milliseconds
import org.firstinspires.ftc.robotcore.external.Telemetry

fun robotPrint(message: Any?) =
	Telemetry::class.objectInstance?.addLine("$message")

fun robotPrintError(message: Any?) =
	Telemetry::class.objectInstance?.addLine("ERROR:$message")

enum class TelemetryLevel {
	Testing, Competition;
}

/**
 * How often (ms) [Telemetry.update] is actually allowed to transmit to the Driver Station.
 *
 * The FTC SDK's default has no throttling
 **/
val TelemetryLevel.transmissionInterval: Time
	get() = when (this) {
		TelemetryLevel.Testing     -> 25.milliseconds
		TelemetryLevel.Competition -> 250.milliseconds
	}

/** Applies [transmissionInterval] for [level] to this [Telemetry]. Call once during `initialize()`. */
fun Telemetry.throttleTo(level: TelemetryLevel) {
	msTransmissionInterval = level.transmissionInterval.asMilliseconds.toInt()
}
