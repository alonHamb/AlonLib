package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.math.control.ProfiledPIDController
import org.firstinspires.ftc.teamcode.alonlib.math.control.TrapezoidProfile

/**
 * Drives [useOutput] with a [ProfiledPIDController] tracking [goal], every scheduler tick. Runs
 * forever -- subclass or [Command.withTimeout] it for an end condition.
 *
 * For a plain position (zero target velocity) goal, pass `{ TrapezoidProfile.State(targetPosition, 0.0) }`.
 */
open class ProfiledPIDCommand(
    protected val controller: ProfiledPIDController,
    private val measurement: () -> Double,
    private val goal: () -> TrapezoidProfile.State,
    private val useOutput: (Double, TrapezoidProfile.State) -> Unit,
    vararg requirements: Subsystem,
) : CommandBase() {

    init {
        addRequirements(*requirements)
    }

    override fun initialize() = controller.reset(measurement())

    override fun execute() = useOutput(controller.calculate(measurement(), goal()), controller.setpoint)

    override fun end(interrupted: Boolean) = useOutput(0.0, TrapezoidProfile.State())
}
