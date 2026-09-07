package org.firstinspires.ftc.teamcode.alonlib.commands

/** Runs [commands] one after another, finishing once the last one has. */
class SequentialCommandGroup(vararg commands: Command) : CommandGroupBase() {

	private val commandList = mutableListOf<Command>()
	private var currentCommandIndex = -1
	private var runWhenDisabled = true

	init {
		addCommands(*commands)
	}

	override fun addCommands(vararg commands: Command) {
		requireUngrouped(*commands)
		check(currentCommandIndex == -1) { "Commands cannot be added to a CommandGroup while the group is running" }
		registerGroupedCommands(*commands)

		for (command in commands) {
			commandList.add(command)
			requirementSet.addAll(command.requirement)
			runWhenDisabled = runWhenDisabled && command.runsWhenDisabled()
		}
	}

	override fun initialize() {
		currentCommandIndex = 0
		commandList.firstOrNull()?.initialize()
	}

	override fun execute() {
		if (commandList.isEmpty() || currentCommandIndex == -1) return

		val current = commandList[currentCommandIndex]
		current.execute()
		if (current.isFinished()) {
			current.end(false)
			currentCommandIndex++
			if (currentCommandIndex < commandList.size) commandList[currentCommandIndex].initialize()
		}
	}

	override fun end(interrupted: Boolean) {
		if (currentCommandIndex == -1) return
		if (interrupted && commandList.isNotEmpty()) commandList[currentCommandIndex].end(true)
		currentCommandIndex = -1
	}

	override fun isFinished() = currentCommandIndex == commandList.size
	override fun runsWhenDisabled() = runWhenDisabled

	val currentCommandName: String get() = commandList[currentCommandIndex].name
}
