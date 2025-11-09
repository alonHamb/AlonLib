package org.firstinspires.ftc.teamcode.alonlib

fun robotPrint(message: Any?, printStackTrace: Boolean = false) =
    print(message.toString())

fun robotPrintError(message: Any?) =
    print(message.toString())

enum class Telemetry {
    Testing, Competition;
}
