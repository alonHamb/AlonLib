package org.firstinspires.ftc.teamcode.alonlib.commands

/** Schedules [command] as non-interruptible, ending once it's no longer scheduled. Wraps a single command -- put several in a group first. */
class UninterruptibleCommand(private val command: Command) : CommandBase() {
    override fun initialize() = command.schedule(interruptible = false)
    override fun isFinished() = !CommandScheduler.isScheduled(command)
}
