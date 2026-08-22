package org.firstinspires.ftc.teamcode.alonlib.commands

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

/** A [LinearOpMode] that drives the [CommandScheduler] for you: init -> init loop -> [preRun] -> repeated [CommandScheduler.run] -> [end]. */
abstract class CommandOpMode : LinearOpMode() {

    fun reset() = CommandScheduler.reset()
    fun run() = CommandScheduler.run()
    fun schedule(vararg commands: Command) = CommandScheduler.schedule(true, *commands)
    fun register(vararg subsystems: Subsystem) = CommandScheduler.registerSubsystem(*subsystems)

    override fun runOpMode() {
        initialize()

        try {
            while (opModeInInit()) initializeLoop()
            if (opModeIsActive()) {
                preRun()
                while (opModeIsActive()) run()
            }
        } finally {
            try {
                end()
            } finally {
                reset()
            }
        }
    }

    abstract fun initialize()

    /** Runs once, after init but before the OpMode goes active. */
    open fun preRun() {}

    /** Runs once the OpMode is no longer active. */
    open fun end() {}

    /** Runs repeatedly during init, like [LinearOpMode]'s own init loop. */
    open fun initializeLoop() {}

    companion object {
        fun disable() = Robot.disable()
        fun enable() = Robot.enable()
    }
}
