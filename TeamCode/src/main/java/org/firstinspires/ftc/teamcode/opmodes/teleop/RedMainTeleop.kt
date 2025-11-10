package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.exampleInstantCommand
import org.firstinspires.ftc.teamcode.commands.printCommand
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class RedMainTeleop : CommandOpMode() {
    var alliance: Alliance = Alliance.Red
    lateinit var controllerA: GamepadEx
    lateinit var controllerB: GamepadEx
    private lateinit var exampleSubsystem: ExampleSubsystem

    override fun initialize() {
        telemetry.isAutoClear = true
        telemetry.addLine("Robot initializing")

        controllerA = GamepadEx(gamepad1)

        controllerB = GamepadEx(gamepad2)

        initSubsystems()

        setDefaultCommands()

        configControllers()

        telemetry.update()
    }

    fun configControllers() {
        with(controllerA) {
            getGamepadButton(GamepadKeys.Button.A).whenHeld(printCommand(controllerA.leftY))
        }
    }

    fun initSubsystems() {
        exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry)
    }

    fun setDefaultCommands() {
        exampleSubsystem.defaultCommand =
            exampleInstantCommand(exampleSubsystem) { controllerA.leftY }
    }

    override fun run() {
        telemetry.update()
        telemetry.addLine("Robot is running")
        super.run()
    }
}