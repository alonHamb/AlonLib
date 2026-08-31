package org.firstinspires.ftc.teamcode.alonlib.commands

import java.util.Collections

/** Runs [commands] simultaneously, finishing once every one of them has (interrupting only the still-running ones if this group itself is interrupted). */
class ParallelCommandGroup(vararg commands: Command) : CommandGroupBase() {

	private val running = LinkedHashMap<Command, Boolean>()
	private var runWhenDisabled = true

	init {
		addCommands(*commands)
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
			if (command.isFinished()) {
				command.end(false)
				running[command] = false
			}
		}
	}

	override fun end(interrupted: Boolean) {
		if (interrupted) {
			for ((command, isRunning) in running) if (isRunning) command.end(true)
		}
	}

	override fun isFinished() = running.values.none { it }
	override fun runsWhenDisabled() = runWhenDisabled
}
