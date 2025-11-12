package org.firstinspires.ftc.teamcode.alonlib

fun robotPrint(message: Any?, printStackTrace: Boolean = false) =
    print("ROBOT PRINT( $message )END ")

fun robotPrintError(message: Any?) =
    print("ROBOT ERROR: $message END")

enum class Telemetry {
    Testing, Competition;
}
