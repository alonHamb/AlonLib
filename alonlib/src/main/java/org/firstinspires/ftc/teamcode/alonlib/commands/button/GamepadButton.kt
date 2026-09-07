package org.firstinspires.ftc.teamcode.alonlib.commands.button

import org.firstinspires.ftc.teamcode.alonlib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.alonlib.gamepad.GamepadKeys

/** A [Button] backed by one or more [GamepadEx] buttons -- active only while all of [buttons] are held. */
class GamepadButton(private val gamepad: GamepadEx, private vararg val buttons: GamepadKeys.Button) : Button() {
    override fun get() = buttons.all { gamepad.getButton(it) }
}
