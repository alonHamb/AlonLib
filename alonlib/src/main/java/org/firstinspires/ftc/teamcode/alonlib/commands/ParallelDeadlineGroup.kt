package org.firstinspires.ftc.teamcode.alonlib.commands

import java.util.Collections

/** Runs [deadline] alongside [commands], ending (and interrupting whatever's still running) as soon as [deadline] finishes. */
class ParallelDeadlineGroup(private var deadline: Command, vararg commands: Command) : CommandGroupBase() {

	private val running = LinkedHashMap<Command, Boolean>()
	private var runWhenDisabled = true

	init {
		addCommands(*commands)
		if (deadline !in running) addCommands(deadline)
	}

	/** Replaces the deadline command, adding it to the group first if it isn't already in it. */
	fun setDeadline(newDeadline: Command) {
		if (newDeadline !in running) addCommands(newDeadline)
		deadline = newDeadline
	}

	override fun addCommands(vararg commands: Command) {
		requireUngrouped(*commands)
		check(running.values.none { it }) { "Commands cannot be added to a CommandGroup while the group is running" }
		registerGroupedCommands(*commands)

		for (command in commands) {
			require(Collections.disjoint(command.requirement, requirementSet)) {
				"Multiple commands in a parallel group cannot require the same subsystems"
			}
			running[command] = false
			requirementSet.addAll(command.requirement)
			runWhenDisabled = runWhenDisabled && command.runsWhenDisabled()
		}
	}

	override fun initialize() {
		for (command in running.keys) {
			command.initialize()
			running[command] = true
		}
	}

	override fun execute() {
		for ((command, isRunning) in running) {
			if (!isRunning) continue
			command.execute()
			if (command !== deadline && command.isFinished()) {
				command.end(false)
				running[command] = false
			}
		}
	}

	override fun end(interrupted: Boolean) {
		for ((command, isRunning) in running) {
			if (command === deadline) command.end(interrupted) else if (isRunning) command.end(true)
		}
	}

	override fun isFinished() = deadline.isFinished()
	override fun runsWhenDisabled() = runWhenDisabled
}
