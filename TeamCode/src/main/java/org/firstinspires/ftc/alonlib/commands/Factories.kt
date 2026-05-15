package org.firstinspires.ftc.teamcode.alonlib.commands

import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.alonlib.units.Mills

fun wait(duration: Mills) = WaitCommand(duration)
fun waitUntil(until: () -> Boolean) = WaitUntilCommand(until)
fun instantCommand(toRun: () -> Unit) = InstantCommand(toRun)

/** THIS COMMAND DOES NOT REQUIRE ANY SUBSYSTEMS. */
val (() -> Unit).asInstantCommand: Command get() = InstantCommand(this)
