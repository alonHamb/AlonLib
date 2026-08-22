package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.units.Time

fun wait(duration: Time) = WaitCommand(duration.asMilliseconds.toLong())
fun waitUntil(until: () -> Boolean) = WaitUntilCommand(until)
fun instantCommand(toRun: () -> Unit) = InstantCommand(toRun)

/** THIS COMMAND DOES NOT REQUIRE ANY SUBSYSTEMS. */
val (() -> Unit).asInstantCommand: Command get() = InstantCommand(this)
