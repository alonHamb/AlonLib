package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/**
 * Runs one of a fixed [commands] map (keyed by whatever [selector] returns), or one built fresh
 * by a [toRun] supplier -- runs the chosen command *through* itself (rather than scheduling it
 * separately), so it behaves correctly nested inside a command group.
 */
class SelectCommand private constructor(
	private val commands: Map<Any, Command>?,
	private val selector: (() -> Any)?,
	private val toRun: (() -> Command)?,
) : CommandBase() {

	constructor(commands: Map<Any, Command>, selector: () -> Any) : this(commands, selector, null) {
		CommandGroupBase.registerGroupedCommands(*commands.values.toTypedArray())
		commands.values.forEach { requirementSet.addAll(it.requirement) }
	}

	constructor(toRun: () -> Command) : this(null, null, toRun)

	private lateinit var selectedCommand: Command

	override fun initialize() {
		selectedCommand = if (selector != null) {
			val key = selector.invoke()
			commands!![key] ?: run {
				robotPrintError("SelectCommand: selector value $key does not correspond to any command")
				InstantCommand()
			}
		} else {
			toRun!!.invoke()
		}
		selectedCommand.initialize()
	}

	override fun execute() = selectedCommand.execute()
	override fun end(interrupted: Boolean) = selectedCommand.end(interrupted)
	override fun isFinished() = selectedCommand.isFinished()

	override fun runsWhenDisabled() =
		commands?.values?.all { it.runsWhenDisabled() } ?: toRun!!.invoke().runsWhenDisabled()
}
