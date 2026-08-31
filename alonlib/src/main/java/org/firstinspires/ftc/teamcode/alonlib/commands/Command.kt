package org.firstinspires.ftc.teamcode.alonlib.commands

/**
 * A state machine representing a complete action to be performed by the robot. Commands are run
 * by the [CommandScheduler], synchronously from the main robot loop, and can be composed into
 * command groups for multistep actions without hand-rolling the state machine.
 */
interface Command {

	/** Called once when the command is initially scheduled. */
	fun initialize() {}

	/** Called repeatedly while the command is scheduled. */
	fun execute() {}

	/** Called once the command finishes (normally or via [interrupted]/[cancel]). */
	fun end(interrupted: Boolean) {}

	/** Once true, the scheduler calls [end] and un-schedules this command. */
	fun isFinished(): Boolean = false

	/**
	 * The subsystems this command needs exclusive use of -- two commands can't use the same one
	 * at once. Prefer returning a stored field over allocating a new set each call.
	 */
	val requirement: Set<Subsystem>

	/** Whether this command requires [requirement]. */
	fun hasRequirement(requirement: Subsystem) = requirement

	/** Whether this command should still run while the robot is disabled. */
	fun runsWhenDisabled(): Boolean = false

	val name: String get() = this::class.simpleName ?: "Command"

	// --- scheduling ---

	fun schedule(interruptible: Boolean = true) = CommandScheduler.schedule(interruptible, this)
	fun cancel() = CommandScheduler.cancel(this)
	fun isScheduled() = CommandScheduler.isScheduled(this)

	// --- decorators (each composes this command into a new one; this command is left untouched) ---

	/** Interrupts and un-schedules the decorated command if it's still running after [millis]. */
	fun withTimeout(millis: Long): Command = ParallelRaceGroup(this, WaitCommand(millis))

	/** Interrupts and un-schedules the decorated command as soon as [condition] becomes true. */
	fun interruptOn(condition: () -> Boolean): Command = ParallelRaceGroup(this, WaitUntilCommand(condition))

	/** Runs [toRun] once this command finishes. */
	fun whenFinished(toRun: () -> Unit): Command = SequentialCommandGroup(this, InstantCommand(toRun))

	/** Runs [toRun] once, before this command starts. */
	fun beforeStarting(toRun: () -> Unit): Command = SequentialCommandGroup(InstantCommand(toRun), this)

	/** Runs [command] to completion, then this command. */
	fun beforeStarting(command: Command): Command = SequentialCommandGroup(command, this)

	/** This command, then [next] in sequence. */
	fun andThen(vararg next: Command): Command = SequentialCommandGroup(this).apply { addCommands(*next) }

	/** This command, interrupting [parallel] once this one ends. */
	fun deadlineWith(vararg parallel: Command): Command = ParallelDeadlineGroup(this, *parallel)

	/** This command alongside [parallel], ending once every one of them has. */
	fun alongWith(vararg parallel: Command): Command = ParallelCommandGroup(this).apply { addCommands(*parallel) }

	/** This command racing [parallel], ending (and interrupting the rest) as soon as any one finishes. */
	fun raceWith(vararg parallel: Command): Command = ParallelRaceGroup(this).apply { addCommands(*parallel) }

	/** This command, ignoring its own end condition -- runs until externally interrupted/canceled. */
	fun perpetually(): Command = PerpetualCommand(this)

	/** This command run "by proxy" ([ProxyScheduleCommand]), so a command group doesn't inherit its requirement. */
	fun asProxy(): Command = ProxyScheduleCommand(this)

	/** This command, refusing to be interrupted by another command sharing a requirement. */
	fun uninterruptible(): Command = UninterruptibleCommand(this)

	/** Runs [runnable] the first time [condition] becomes true while this command is active. */
	fun whenActive(condition: () -> Boolean, runnable: () -> Unit): Command = CallbackCommand(this).apply { whenTrue(condition, runnable) }

	/** Schedules [command] the first time [condition] becomes true while this command is active. */
	fun whenActive(condition: () -> Boolean, command: Command): Command = CallbackCommand(this).apply { whenTrue(condition, command) }
}
