package org.firstinspires.ftc.teamcode.alonlib.commands

/** Does nothing until [condition] becomes true. */
open class WaitUntilCommand(private val condition: () -> Boolean) : CommandBase() {
    override fun isFinished() = condition()
    override fun runsWhenDisabled() = true
}
