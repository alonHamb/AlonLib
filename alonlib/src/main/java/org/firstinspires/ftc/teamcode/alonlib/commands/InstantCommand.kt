package org.firstinspires.ftc.teamcode.alonlib.commands

/** A command that runs [toRun] once, then finishes immediately (initializes, executes, and ends on the same scheduler tick). */
open class InstantCommand(private val toRun: () -> Unit = {}, vararg requirements: Subsystem) : CommandBase() {

    init {
        addRequirements(*requirements)
    }

    override fun initialize() = toRun()
    final override fun isFinished() = true
}
