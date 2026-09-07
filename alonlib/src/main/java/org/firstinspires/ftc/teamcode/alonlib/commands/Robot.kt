package org.firstinspires.ftc.teamcode.alonlib.commands

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

/** Thin convenience wrapper around the [CommandScheduler] singleton, plus the global disabled flag it checks. */
object Robot {

    var isDisabled = false

    fun reset() = CommandScheduler.reset()
    fun run() = CommandScheduler.run()
    fun schedule(vararg commands: Command) = CommandScheduler.schedule(true, *commands)
    fun register(vararg subsystems: Subsystem) = CommandScheduler.registerSubsystem(*subsystems)
    fun disable() { isDisabled = true }
    fun enable() { isDisabled = false }

    fun setBulkReading(hardwareMap: HardwareMap, cachingMode: LynxModule.BulkCachingMode) =
        CommandScheduler.setBulkReading(hardwareMap, cachingMode)
}
