package alonlib.commands

/** Runs [toRun] every iteration, with no end condition of its own -- pair with [alonlib.commands.withTimeout]/[Command.interruptOn], or use [InstantCommand] for a one-shot. */
open class RunCommand(private val toRun: () -> Unit, vararg requirements: Subsystem) : CommandBase() {

    init {
        addRequirements(*requirements)
    }

    override fun execute() = toRun()
}
