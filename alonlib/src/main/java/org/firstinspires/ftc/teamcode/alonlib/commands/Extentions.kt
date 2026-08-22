package org.firstinspires.ftc.teamcode.alonlib.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.command.WaitUntilCommand

/**
 * solverslib's `Command` has no `until`/`finallyDo`/`withName` members and only a millisecond
 * overload of `withTimeout` -- so, unlike [andThen]/[alongWith]/[raceWith] below (which really do
 * delegate to real members), the four extensions past this comment can't just call `this.xxx(...)`:
 * with no matching member in scope, that resolves right back to the extension itself and blows the
 * stack the instant it's called. (This bit AlonLib itself: all four were exactly that before this
 * fix, caught by [org.firstinspires.ftc.teamcode.alonlib.commands.ExtentionsTest].)
 */
infix fun Command.until(condition: () -> Boolean): Command = this.raceWith(WaitUntilCommand { condition() })
infix fun Command.andThen(next: Command): Command = this.andThen(next)
infix fun Command.andThen(next: () -> Command): Command = this.andThen(next())

private class FinallyDoCommand(private val command: Command, private val onEnd: (interrupted: Boolean) -> Unit) : CommandBase() {
    init {
        addRequirements(*command.requirements.toTypedArray())
    }

    override fun initialize() = command.initialize()
    override fun execute() = command.execute()
    override fun isFinished() = command.isFinished()
    override fun end(interrupted: Boolean) {
        command.end(interrupted)
        onEnd(interrupted)
    }
}

infix fun Command.finallyDo(end: (interrupted: Boolean) -> Unit): Command = FinallyDoCommand(this, end)
infix fun Command.finallyDo(command: Command): Command = this.finallyDo { _ -> command.schedule() }

infix fun Command.alongWith(parallel: Command): Command = this.alongWith(parallel)
infix fun Command.raceWith(parallel: Command): Command = this.raceWith(parallel)

/** solverslib's own `withTimeout` takes milliseconds -- see the doc comment above [until]. */
infix fun Command.withTimeout(seconds: Double): Command = this.withTimeout((seconds * 1000.0).toLong())

/**
 * Good for multi-subsystem commands.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
infix fun CommandBase.withName(commandName: String): Command = this.setName(commandName)

/**
 * Good for multi-subsystem commands.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
fun withName(commandName: String, commandSupplier: () -> CommandBase): Command =
    commandSupplier().also { it.name = commandName }

/**
 * Good for single-subsystem commands.
 * Appends the name of the subsystem to the String in [commandName : subsystemName] format.
 *
 * For multi-subsystem commands, use [withName].
 */
fun SubsystemBase.withName(commandName: String, commandSupplier: () -> CommandBase): Command =
    commandSupplier().also { it.name = "$commandName : ${this.name}" }
