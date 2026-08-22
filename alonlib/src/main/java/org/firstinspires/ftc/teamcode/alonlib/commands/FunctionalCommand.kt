package org.firstinspires.ftc.teamcode.alonlib.commands

/** Assembles a [Command] from plain lambdas for each lifecycle method -- handy for a one-off command not worth its own class. */
open class FunctionalCommand(
    private val onInit: () -> Unit,
    private val onExecute: () -> Unit,
    private val onEnd: (interrupted: Boolean) -> Unit,
    private val isFinishedFn: () -> Boolean,
    vararg requirements: Subsystem,
) : CommandBase() {

    init {
        addRequirements(*requirements)
    }

    override fun initialize() = onInit()
    override fun execute() = onExecute()
    override fun end(interrupted: Boolean) = onEnd(interrupted)
    override fun isFinished() = isFinishedFn()
}
