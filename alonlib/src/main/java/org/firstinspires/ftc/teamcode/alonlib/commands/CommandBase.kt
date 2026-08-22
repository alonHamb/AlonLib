package org.firstinspires.ftc.teamcode.alonlib.commands

/** A base class for [Command]s, tracking a name/subsystem-group label and a set of [requirements]. */
abstract class CommandBase : Command {

    override var name: String = this::class.simpleName ?: "Command"
    var subsystemGroup: String = "Ungrouped"

    /** Backing set for [requirements] -- protected so command-group subclasses can union in their component commands' requirements. */
    protected val requirementsSet = mutableSetOf<Subsystem>()
    override val requirements: Set<Subsystem> get() = requirementsSet

    fun addRequirements(vararg requirements: Subsystem) = apply { requirementsSet.addAll(requirements) }

    /** Matches SolversLib's `setName`/`setSubsystem` method-call API (as opposed to the [name]/[subsystemGroup] properties). */
    fun setName(newName: String) = apply { name = newName }
    fun setSubsystem(newSubsystem: String) = apply { subsystemGroup = newSubsystem }
}
