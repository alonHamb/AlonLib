package org.firstinspires.ftc.teamcode.alonlib.commands

import java.util.Collections
import java.util.WeakHashMap

/**
 * A base for command groups. Statically tracks every command that's been allocated to a group, so
 * that a grouped command can't also be scheduled independently (which would leave its state
 * inconsistent between the two).
 */
abstract class CommandGroupBase : CommandBase() {

    /** Adds [commands] to this group. */
    abstract fun addCommands(vararg commands: Command)

    companion object {
        private val groupedCommands: MutableSet<Command> = Collections.newSetFromMap(WeakHashMap())

        internal fun registerGroupedCommands(vararg commands: Command) {
            groupedCommands.addAll(commands)
        }

        /** Frees every grouped command to be scheduled independently again. Use with care. */
        fun clearGroupedCommands() = groupedCommands.clear()

        /** Frees [command] to be scheduled independently again. Use with care. */
        fun clearGroupedCommand(command: Command) {
            groupedCommands.remove(command)
        }

        /** Throws if any of [commands] is already part of another command group. */
        fun requireUngrouped(vararg commands: Command) = requireUngrouped(commands.toList())

        fun requireUngrouped(commands: Collection<Command>) {
            require(Collections.disjoint(commands, groupedCommands)) { "Commands cannot be added to more than one CommandGroup" }
        }

        internal fun getGroupedCommands(): Set<Command> = groupedCommands
    }
}
