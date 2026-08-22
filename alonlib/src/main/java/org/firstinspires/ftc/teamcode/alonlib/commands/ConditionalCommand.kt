package org.firstinspires.ftc.teamcode.alonlib.commands

/**
 * Runs [onTrue] or [onFalse] depending on [condition] at the moment this command is initialized
 * -- it runs the chosen command *through* itself (rather than scheduling it separately), so it
 * behaves correctly nested inside a command group. Requires the union of both branches'
 * requirements, for the same reason.
 */
class ConditionalCommand(private val onTrue: Command, private val onFalse: Command, private val condition: () -> Boolean) : CommandBase() {

    private lateinit var selectedCommand: Command

    init {
        CommandGroupBase.requireUngrouped(onTrue, onFalse)
        CommandGroupBase.registerGroupedCommands(onTrue, onFalse)
        requirementsSet.addAll(onTrue.requirements)
        requirementsSet.addAll(onFalse.requirements)
    }

    override fun initialize() {
        selectedCommand = if (condition()) onTrue else onFalse
        selectedCommand.initialize()
    }

    override fun execute() = selectedCommand.execute()
    override fun end(interrupted: Boolean) = selectedCommand.end(interrupted)
    override fun isFinished() = selectedCommand.isFinished()
    override fun runsWhenDisabled() = onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled()
}
