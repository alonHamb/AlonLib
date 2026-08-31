package org.firstinspires.ftc.teamcode.alonlib.commands

import java.util.Collections

/** Runs [commands] simultaneously, ending (and interrupting the rest) as soon as any one of them finishes. */
class ParallelRaceGroup(vararg commands: Command) : CommandGroupBase() {

	private val commandSet = mutableSetOf<Command>()
	private var runWhenDisabled = true
	private var finished = true

	init {
		addCommands(*commands)
	}

	override fun addCommands(vararg commands: Command) {
		requireUngrouped(*commands)
		check(finished) { "Commands cannot be added to a CommandGroup while the group is running" }
		registerGroupedCommands(*commands)

		for (command in commands) {
			require(Collections.disjoint(command.requirement, requirementSet)) {
				"Multiple commands in a parallel group cannot require the same subsystems"
			}
			commandSet.add(command)
			requirementSet.addAll(command.requirement)
			runWhenDisabled = runWhenDisabled && command.runsWhenDisabled()
		}
	}

	override fun initialize() {
		finished = false
		commandSet.forEach { it.initialize() }
	}

	override fun execute() {
		for (command in commandSet) {
			command.execute()
			if (command.isFinished()) finished = true
		}
	}

	override fun end(interrupted: Boolean) {
		for (command in commandSet) if (!command.isFinished()) command.end(true)
	}

	override fun isFinished() = finished
	override fun runsWhenDisabled() = runWhenDisabled
}
