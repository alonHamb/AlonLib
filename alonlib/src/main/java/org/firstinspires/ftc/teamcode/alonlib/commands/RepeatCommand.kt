package org.firstinspires.ftc.teamcode.alonlib.commands

/**
 * Runs [command] repeatedly, restarting it each time it finishes, until this command is
 * interrupted -- or, if given, until [maxRepeatTimes] repeats or [condition] becomes true.
 */
class RepeatCommand(private val command: Command, private val maxRepeatTimes: Int = 0, private val condition: (() -> Boolean)? = null) : CommandBase() {

    private var timesRepeated = 0

    init {
        require(maxRepeatTimes >= 0) { "RepeatCommand's maxRepeatTimes cannot be negative!" }
        CommandGroupBase.requireUngrouped(command)
        CommandGroupBase.registerGroupedCommands(command)
        requirementsSet.addAll(command.requirements)
    }

    override fun initialize() {
        timesRepeated = 0
        command.initialize()
    }

    override fun execute() {
        command.execute()
        if (command.isFinished()) {
            command.end(false)
            timesRepeated++
            if (!isFinished()) command.initialize()
        }
    }

    override fun isFinished() = (maxRepeatTimes > 0 && timesRepeated >= maxRepeatTimes) || condition?.invoke() == true

    override fun end(interrupted: Boolean) {
        if (command.isScheduled() || !command.isFinished()) command.end(interrupted)
    }

    override fun runsWhenDisabled() = command.runsWhenDisabled()
}
