package org.firstinspires.ftc.teamcode.alonlib.commands

/**
 * Defers building the actual command until this one is initialized -- useful for runtime-dependent
 * setup (e.g. building a trajectory mid-autonomous). [supplier] must build a *new* [Command] every
 * call; for picking among a fixed set instead, use [SelectCommand].
 */
class DeferredCommand(private val supplier: () -> Command, requirements: Collection<Subsystem> = emptySet()) : CommandBase() {

    private var command: Command? = null

    init {
        requirementsSet.addAll(requirements)
    }

    override fun initialize() {
        val built = supplier()
        command = built
        built.initialize()
    }

    override fun execute() = command?.execute() ?: Unit

    override fun isFinished() = command?.isFinished() ?: true

    override fun end(interrupted: Boolean) {
        command?.end(interrupted)
        command = null
    }
}
