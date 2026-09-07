package org.firstinspires.ftc.teamcode.alonlib.commands

/** Prints [message] to stdout once, then finishes. Runs even while disabled. */
class PrintCommand(message: String) : InstantCommand({ println(message) }) {
    override fun runsWhenDisabled() = true
}
