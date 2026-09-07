package org.firstinspires.ftc.teamcode.alonlib.commands

/** Runs [command] forever, ignoring its own end condition -- only external interruption/cancellation stops it. */
class PerpetualCommand(private val command: Command) : CommandBase() {

	init {
		CommandGroupBase.requireUngrouped(command)
		CommandGroupBase.registerGroupedCommands(command)
		requirementSet.addAll(command.requirement)
	}

	override fun initialize() = command.initialize()
	override fun execute() = command.execute()
	override fun end(interrupted: Boolean) = command.end(interrupted)
	override fun runsWhenDisabled() = command.runsWhenDisabled()
}
