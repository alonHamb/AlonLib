package org.firstinspires.ftc.teamcode.alonlib.commands

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Collections

/**
 * Runs [Command]s. Call [run] every robot loop iteration to drive commands synchronously.
 * [registerSubsystem] a [Subsystem] so its [Subsystem.periodic] runs and its default command gets
 * scheduled.
 */
object CommandScheduler {

    private val scheduledCommands = LinkedHashMap<Command, CommandState>()
    private val requirementOwners = LinkedHashMap<Subsystem, Command>()
    private val subsystems = LinkedHashMap<Subsystem, Command?>()
    private val buttons = LinkedHashSet<() -> Unit>()

    private var disabled = false

    private val initActions = mutableListOf<(Command) -> Unit>()
    private val executeActions = mutableListOf<(Command) -> Unit>()
    private val interruptActions = mutableListOf<(Command) -> Unit>()
    private val finishActions = mutableListOf<(Command) -> Unit>()

    private val toSchedule = LinkedHashMap<Command, Boolean>()
    private var inRunLoop = false
    private val toCancel = mutableListOf<Command>()
    private var clearHubCacheEachLoop = false
    private var allHubs: List<LynxModule> = emptyList()

    /** Adds a button binding that gets polled every [run] to (maybe) schedule a command. */
    fun addButton(button: () -> Unit) {
        buttons.add(button)
    }

    fun clearButtons() = buttons.clear()

    private fun initCommand(command: Command, interruptible: Boolean, requirements: Set<Subsystem>) {
        command.initialize()
        scheduledCommands[command] = CommandState(interruptible)
        initActions.forEach { it(command) }
        for (requirement in requirements) requirementOwners[requirement] = command
    }

    private fun scheduleOne(interruptible: Boolean, command: Command) {
        if (inRunLoop) {
            toSchedule[command] = interruptible
            return
        }

        require(command !in CommandGroupBase.getGroupedCommands()) {
            "A command that is part of a command group cannot be independently scheduled"
        }

        if (disabled || (!command.runsWhenDisabled() && Robot.isDisabled) || command in scheduledCommands) return

        val requirements = command.requirements

        if (Collections.disjoint(requirementOwners.keys, requirements)) {
            initCommand(command, interruptible, requirements)
        } else {
            for (requirement in requirements) {
                val owner = requirementOwners[requirement]
                if (owner != null && scheduledCommands[owner]?.isInterruptible == false) return
            }
            for (requirement in requirements) {
                requirementOwners[requirement]?.let { cancel(it) }
            }
            initCommand(command, interruptible, requirements)
        }
    }

    /** Schedules [commands], as [interruptible] if their requirements are currently in use. No-op for an already-scheduled command. */
    fun schedule(interruptible: Boolean = true, vararg commands: Command) {
        commands.forEach { scheduleOne(interruptible, it) }
    }

    /**
     * Runs one scheduler iteration: subsystem [Subsystem.periodic]s, button bindings, every
     * scheduled command's [Command.execute] (removing those that finish), then default commands
     * for any subsystem left unrequired.
     */
    fun run() {
        if (disabled) return

        for (subsystem in subsystems.keys) subsystem.periodic()
        for (button in buttons) button()

        inRunLoop = true
        val iterator = scheduledCommands.keys.iterator()
        while (iterator.hasNext()) {
            val command = iterator.next()

            if (!command.runsWhenDisabled() && Robot.isDisabled) {
                command.end(true)
                interruptActions.forEach { it(command) }
                requirementOwners.keys.removeAll(command.requirements)
                iterator.remove()
                continue
            }

            command.execute()
            executeActions.forEach { it(command) }
            if (command.isFinished()) {
                command.end(false)
                finishActions.forEach { it(command) }
                iterator.remove()
                requirementOwners.keys.removeAll(command.requirements)
            }
        }
        inRunLoop = false

        for ((command, interruptible) in toSchedule) scheduleOne(interruptible, command)
        for (command in toCancel) cancel(command)
        toSchedule.clear()
        toCancel.clear()

        for ((subsystem, defaultCommand) in subsystems) {
            if (subsystem !in requirementOwners && defaultCommand != null) schedule(true, defaultCommand)
        }

        if (clearHubCacheEachLoop) allHubs.forEach { it.clearBulkCache() }
    }

    /** Registers [subsystems] so their [Subsystem.periodic] runs and their default command can be scheduled. */
    fun registerSubsystem(vararg subsystems: Subsystem) {
        for (subsystem in subsystems) this.subsystems[subsystem] = null
    }

    fun unregisterSubsystem(vararg subsystems: Subsystem) = this.subsystems.keys.removeAll(subsystems.toSet())

    /** Sets every registered hub's bulk-caching mode. MANUAL is recommended -- it needs no extra work and never re-reads within a loop. */
    fun setBulkReading(hardwareMap: HardwareMap, cachingMode: LynxModule.BulkCachingMode) {
        allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { it.bulkCachingMode = cachingMode }
        clearHubCacheEachLoop = cachingMode == LynxModule.BulkCachingMode.MANUAL
    }

    /** Clears all scheduler state. Mainly useful between OpModes/tests. */
    fun reset() {
        scheduledCommands.clear()
        requirementOwners.clear()
        subsystems.clear()
        buttons.clear()
        initActions.clear()
        executeActions.clear()
        interruptActions.clear()
        finishActions.clear()
        toSchedule.clear()
        toCancel.clear()
        inRunLoop = false
        disabled = false
    }

    /** The default command for [subsystem], automatically scheduled whenever nothing else requires it. It must itself require [subsystem] and never finish. */
    fun setDefaultCommand(subsystem: Subsystem, defaultCommand: Command) {
        require(subsystem in defaultCommand.requirements) { "Default commands must require their subsystem!" }
        require(!defaultCommand.isFinished()) { "Default commands should not end!" }
        subsystems[subsystem] = defaultCommand
    }

    fun getDefaultCommand(subsystem: Subsystem): Command? = subsystems[subsystem]

    /** Every command currently scheduled directly by the scheduler (not commands nested inside a running group). */
    fun getScheduledCommands(): List<Command> = scheduledCommands.keys.toList()

    /** Interrupts and un-schedules [commands], even ones scheduled as non-interruptible. Calls [Command.end] with `interrupted = true`. */
    fun cancel(vararg commands: Command) {
        if (inRunLoop) {
            toCancel.addAll(commands)
            return
        }

        for (command in commands) {
            if (command !in scheduledCommands) continue
            command.end(true)
            interruptActions.forEach { it(command) }
            scheduledCommands.remove(command)
            requirementOwners.keys.removeAll(command.requirements)
        }
    }

    fun cancelAll() = cancel(*scheduledCommands.keys.toTypedArray())

    /** Whether every one of [commands] is directly scheduled (nested commands inside a group aren't visible here). */
    fun isScheduled(vararg commands: Command): Boolean = scheduledCommands.keys.containsAll(commands.toList())

    /** The command currently requiring [subsystem], or null if it's free. */
    fun requiring(subsystem: Subsystem): Command? = requirementOwners[subsystem]

    fun isAvailable(subsystem: Subsystem) = requiring(subsystem) == null

    fun disable() { disabled = true }
    fun enable() { disabled = false }

    fun onCommandInitialize(action: (Command) -> Unit) = initActions.add(action)
    fun onCommandExecute(action: (Command) -> Unit) = executeActions.add(action)
    fun onCommandInterrupt(action: (Command) -> Unit) = interruptActions.add(action)
    fun onCommandFinish(action: (Command) -> Unit) = finishActions.add(action)
}
