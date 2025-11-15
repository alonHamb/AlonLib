package org.firstinspires.ftc.teamcode.alonlib

fun robotPrint(message: Any?) =
    print("ROBOT PRINT( $message )END ")

fun robotPrintError(message: Any?) =
    print("ROBOT ERROR: $message END")

enum class Telemetry {
    Testing, Competition;
}
