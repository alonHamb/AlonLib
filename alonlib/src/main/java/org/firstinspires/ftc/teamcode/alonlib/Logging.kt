package org.firstinspires.ftc.teamcode.alonlib

import org.firstinspires.ftc.robotcore.external.Telemetry

fun robotPrint(message: Any?) =
    print("ROBOT PRINT( $message )END ")

fun robotPrintError(message: Any?) =
    print("ROBOT ERROR: $message END")

enum class TelemetryLevel {
    Testing, Competition;
}

/**
 * How often (ms) [Telemetry.update] is actually allowed to transmit to the Driver Station.
 *
 * The FTC SDK's default has no throttling -- every [Telemetry.update] call transmits over the DS
 * radio link immediately, so calling it once per loop (the normal pattern) still means one
 * transmission per loop, which is a common cause of loop time growing. Setting
 * [Telemetry.msTransmissionInterval] (see [throttleTo]) lets `update()` keep being called every
 * loop cheaply while the actual send is rate-limited to this interval.
 */
val TelemetryLevel.transmissionIntervalMs: Int
    get() = when (this) {
        TelemetryLevel.Testing -> 25
        TelemetryLevel.Competition -> 250
    }

/** Applies [transmissionIntervalMs] for [level] to this [Telemetry]. Call once during `initialize()`. */
fun Telemetry.throttleTo(level: TelemetryLevel) {
    msTransmissionInterval = level.transmissionIntervalMs
}
