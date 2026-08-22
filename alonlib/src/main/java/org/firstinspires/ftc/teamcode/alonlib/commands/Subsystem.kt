package org.firstinspires.ftc.teamcode.alonlib.commands

/**
 * A robot subsystem: encapsulates a piece of hardware and exposes it to [Command]s. The
 * [CommandScheduler] uses each command's [Command.requirements] to make sure only one command
 * uses a given subsystem at a time.
 */
interface Subsystem {

    /** Called every scheduler tick, for subsystem-internal state that shouldn't live in a [Command]. */
    fun periodic() {}

    fun setDefaultCommand(defaultCommand: Command) = CommandScheduler.setDefaultCommand(this, defaultCommand)
    fun defaultCommand(): Command? = CommandScheduler.getDefaultCommand(this)
    fun currentCommand(): Command? = CommandScheduler.requiring(this)

    /** Registers this subsystem with the [CommandScheduler] so [periodic] gets called and its default command can run. */
    fun register() = CommandScheduler.registerSubsystem(this)

    /** A command that runs [action] once and finishes, requiring this subsystem. */
    fun runOnce(action: () -> Unit): Command = InstantCommand(action, this)

    /** A command that runs [action] every iteration until interrupted, requiring this subsystem. */
    fun run(action: () -> Unit): Command = RunCommand(action, this)

    /** A command that runs [start] once, then [end] when interrupted, requiring this subsystem. */
    fun startEnd(start: () -> Unit, end: () -> Unit): Command = StartEndCommand(start, end, this)

    /** A command that runs [run] every iteration until interrupted, then [end], requiring this subsystem. */
    fun runEnd(run: () -> Unit, end: () -> Unit): Command =
        FunctionalCommand({}, run, { _ -> end() }, { false }, this)

    /** A command that runs [start] once, then [run] every iteration until interrupted, requiring this subsystem. */
    fun startRun(start: () -> Unit, run: () -> Unit): Command =
        FunctionalCommand(start, run, { _ -> }, { false }, this)

    /** A [DeferredCommand] built from [supplier] at schedule time, requiring this subsystem. */
    fun defer(supplier: () -> Command): Command = DeferredCommand(supplier, setOf(this))
}
