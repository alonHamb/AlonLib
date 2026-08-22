package org.firstinspires.ftc.teamcode.alonlib.commands.button

import org.firstinspires.ftc.teamcode.alonlib.commands.Command
import org.firstinspires.ftc.teamcode.alonlib.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.alonlib.commands.InstantCommand

/**
 * Links commands to a polled boolean condition (e.g. a gamepad button). Subclass and override
 * [get] for anything beyond a plain condition (see [org.firstinspires.ftc.teamcode.alonlib.commands.button.GamepadButton]).
 */
open class Trigger(private val isActive: () -> Boolean = { false }) {

    /** Whether the trigger is currently active. Called every scheduler tick once bound to a command. */
    open fun get(): Boolean = isActive()

    /** Schedules [command] the moment the trigger goes from inactive to active. */
    fun whenActive(command: Command, interruptible: Boolean = true) = apply {
        CommandScheduler.addButton(edgeTriggered(rising = true) { command.schedule(interruptible) })
    }

    fun whenActive(toRun: () -> Unit) = whenActive(InstantCommand(toRun))

    /** Re-schedules [command] every tick the trigger is active, and cancels it the moment it isn't. */
    fun whileActiveContinuous(command: Command, interruptible: Boolean = true) = apply {
        CommandScheduler.addButton(object : () -> Unit {
            var pressedLast = get()
            override fun invoke() {
                val pressed = get()
                if (pressed) command.schedule(interruptible) else if (pressedLast) command.cancel()
                pressedLast = pressed
            }
        })
    }

    fun whileActiveContinuous(toRun: () -> Unit) = whileActiveContinuous(InstantCommand(toRun))

    /** Schedules [command] once the trigger becomes active, canceling it once the trigger goes inactive -- never re-scheduling it in between. */
    fun whileActiveOnce(command: Command, interruptible: Boolean = true) = apply {
        CommandScheduler.addButton(object : () -> Unit {
            var pressedLast = get()
            override fun invoke() {
                val pressed = get()
                if (!pressedLast && pressed) command.schedule(interruptible) else if (pressedLast && !pressed) command.cancel()
                pressedLast = pressed
            }
        })
    }

    /** Schedules [command] the moment the trigger goes from active to inactive. */
    fun whenInactive(command: Command, interruptible: Boolean = true) = apply {
        CommandScheduler.addButton(edgeTriggered(rising = false) { command.schedule(interruptible) })
    }

    fun whenInactive(toRun: () -> Unit) = whenInactive(InstantCommand(toRun))

    /** Toggles [command] on/off each time the trigger goes from inactive to active. */
    fun toggleWhenActive(command: Command, interruptible: Boolean = true) = apply {
        CommandScheduler.addButton(edgeTriggered(rising = true) {
            if (command.isScheduled()) command.cancel() else command.schedule(interruptible)
        })
    }

    /** Alternates between [commandOne] and [commandTwo] each time the trigger goes from inactive to active. */
    fun toggleWhenActive(commandOne: Command, commandTwo: Command, interruptible: Boolean = true) = apply {
        CommandScheduler.addButton(object : () -> Unit {
            var pressedLast = get()
            var firstCommandActive = false
            override fun invoke() {
                val pressed = get()
                if (!pressedLast && pressed) {
                    if (firstCommandActive) {
                        if (commandOne.isScheduled()) commandOne.cancel()
                        commandTwo.schedule(interruptible)
                    } else {
                        if (commandTwo.isScheduled()) commandTwo.cancel()
                        commandOne.schedule(interruptible)
                    }
                    firstCommandActive = !firstCommandActive
                }
                pressedLast = pressed
            }
        })
    }

    fun toggleWhenActive(runnableOne: () -> Unit, runnableTwo: () -> Unit) =
        toggleWhenActive(InstantCommand(runnableOne), InstantCommand(runnableTwo))

    /** Cancels [command] the moment the trigger goes from inactive to active. */
    fun cancelWhenActive(command: Command) = apply {
        CommandScheduler.addButton(edgeTriggered(rising = true) { command.cancel() })
    }

    /** A trigger active only when both this and [other] are. */
    infix fun and(other: Trigger) = Trigger { get() && other.get() }

    /** A trigger active whenever either this or [other] is. */
    infix fun or(other: Trigger) = Trigger { get() || other.get() }

    /** A trigger active exactly when this one isn't. */
    fun negate() = Trigger { !get() }

    private fun edgeTriggered(rising: Boolean, action: () -> Unit): () -> Unit {
        var pressedLast = get()
        return {
            val pressed = get()
            if (rising) {
                if (!pressedLast && pressed) action()
            } else {
                if (pressedLast && !pressed) action()
            }
            pressedLast = pressed
        }
    }
}
