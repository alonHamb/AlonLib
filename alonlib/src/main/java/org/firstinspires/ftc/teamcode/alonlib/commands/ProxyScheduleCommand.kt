package org.firstinspires.ftc.teamcode.alonlib.commands

/** Schedules [toSchedule] independently (not as requirements of this command) when initialized, ending once none of them are scheduled anymore. Cancels them all if interrupted. */
class ProxyScheduleCommand(private vararg val toSchedule: Command) : CommandBase() {

    private var finished = false

    override fun initialize() {
        toSchedule.forEach { it.schedule() }
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) toSchedule.forEach { it.cancel() }
    }

    override fun execute() {
        finished = toSchedule.all { !it.isScheduled() }
    }

    override fun isFinished() = finished
}
