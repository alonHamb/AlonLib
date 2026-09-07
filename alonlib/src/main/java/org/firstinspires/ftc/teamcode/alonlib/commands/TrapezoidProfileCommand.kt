package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.math.control.TrapezoidProfile

/** Runs a [TrapezoidProfile] from [initial] towards [goal], piping each step's state to [output], until the profile completes. */
class TrapezoidProfileCommand(
    constraints: TrapezoidProfile.Constraints,
    private val goal: TrapezoidProfile.State,
    private val initial: TrapezoidProfile.State = TrapezoidProfile.State(),
    private val output: (TrapezoidProfile.State) -> Unit,
    vararg requirements: Subsystem,
) : CommandBase() {

    private val profile = TrapezoidProfile(constraints)
    private var startNanos = 0L

    init {
        addRequirements(*requirements)
    }

    override fun initialize() {
        startNanos = System.nanoTime()
    }

    override fun execute() = output(profile.calculate(elapsedSeconds(), initial, goal))

    override fun isFinished() = profile.isFinished(elapsedSeconds())

    private fun elapsedSeconds() = (System.nanoTime() - startNanos) / 1e9
}
