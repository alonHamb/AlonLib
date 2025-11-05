package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.commands.TestCommand

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {
    val map = hardwareMap
    override fun initialize() {

        telemetry.addLine("init")
        telemetry.update()

        val controllerA = GamepadEx(gamepad1)

        controllerA.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(TestCommand(1.0))
    }

    override fun run() {
        super.run()
        telemetry.addLine("run")
        telemetry.update()
    }
}