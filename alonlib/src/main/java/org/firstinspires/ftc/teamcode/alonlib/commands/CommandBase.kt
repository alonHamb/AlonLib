package org.firstinspires.ftc.teamcode.alonlib.commands

/** A base class for [Command]s, tracking a name/subsystem-group label and a set of [requirement]. */
abstract class CommandBase : Command {

	override var name: String = this::class.simpleName ?: "Command"
	var subsystemGroup: String = "Ungrouped"

	/** Backing set for [requirement] -- protected so command-group subclasses can union in their component commands' requirements. */
	protected val requirementSet = mutableSetOf<Subsystem>()
	override val requirement: Set<Subsystem> get() = requirementSet

	fun addRequirements(vararg requirements: Subsystem) = apply { requirementSet.addAll(requirements) }

	/** Matches SolversLib's `setName`/`setSubsystem` method-call API (as opposed to the [name]/[subsystemGroup] properties). */
	fun setName(newName: String) = apply { name = newName }
	fun setSubsystem(newSubsystem: String) = apply { subsystemGroup = newSubsystem }
}
