package org.firstinspires.ftc.teamcode.alonlib.commands

/** Runs [onInit] once at start and [onEnd] once at end (e.g. spin up a motor, then stop it). No end condition of its own. */
open class StartEndCommand(private val onInit: () -> Unit, private val onEnd: () -> Unit, vararg requirements: Subsystem) : CommandBase() {

    init {
        addRequirements(*requirements)
    }

    override fun initialize() = onInit()
    override fun end(interrupted: Boolean) = onEnd()
}
