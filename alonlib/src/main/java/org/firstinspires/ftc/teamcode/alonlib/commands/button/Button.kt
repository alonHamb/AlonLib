package org.firstinspires.ftc.teamcode.alonlib.commands.button

import org.firstinspires.ftc.teamcode.alonlib.commands.Command

/** A [Trigger] specifically for an operator-interface button -- same behavior, renamed to fit a button's use case. */
abstract class Button(isPressed: () -> Boolean = { false }) : Trigger(isPressed) {

    fun whenPressed(command: Command, interruptible: Boolean = true) = apply { whenActive(command, interruptible) }
    fun whenPressed(toRun: () -> Unit) = apply { whenActive(toRun) }

    fun whileHeld(command: Command, interruptible: Boolean = true) = apply { whileActiveContinuous(command, interruptible) }
    fun whileHeld(toRun: () -> Unit) = apply { whileActiveContinuous(toRun) }

    fun whenHeld(command: Command, interruptible: Boolean = true) = apply { whileActiveOnce(command, interruptible) }

    fun whenReleased(command: Command, interruptible: Boolean = true) = apply { whenInactive(command, interruptible) }
    fun whenReleased(toRun: () -> Unit) = apply { whenInactive(toRun) }

    fun toggleWhenPressed(command: Command, interruptible: Boolean = true) = apply { toggleWhenActive(command, interruptible) }
    fun toggleWhenPressed(commandOne: Command, commandTwo: Command, interruptible: Boolean = true) =
        apply { toggleWhenActive(commandOne, commandTwo, interruptible) }
    fun toggleWhenPressed(runnableOne: () -> Unit, runnableTwo: () -> Unit) = apply { toggleWhenActive(runnableOne, runnableTwo) }

    fun cancelWhenPressed(command: Command) = apply { cancelWhenActive(command) }
}
