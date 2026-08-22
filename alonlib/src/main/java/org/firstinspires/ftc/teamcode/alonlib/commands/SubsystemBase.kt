package org.firstinspires.ftc.teamcode.alonlib.commands

/** A base for [Subsystem]s: auto-registers with the [CommandScheduler] and tracks a display [name]. */
abstract class SubsystemBase : Subsystem {

    var name: String = this::class.simpleName ?: "Subsystem"
    var subsystemGroup: String
        get() = name
        set(value) { name = value }

    init {
        register()
    }
}
