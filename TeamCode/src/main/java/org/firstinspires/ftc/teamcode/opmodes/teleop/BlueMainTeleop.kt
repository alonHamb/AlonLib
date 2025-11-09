package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.commands.printCommand
import org.firstinspires.ftc.teamcode.commands.testCommand
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {

    private lateinit var testSubsystem: TestSubsystem
    lateinit var controllerA: GamepadEx
    lateinit var controllerB: GamepadEx

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
        testSubsystem = TestSubsystem(hardwareMap, telemetry)
    }

    fun setDefaultCommands() {
        testSubsystem.defaultCommand = testCommand(testSubsystem, { controllerA.leftY })
    }

    override fun run() {
        telemetry.update()
        telemetry.addLine("Robot is running")
        super.run()
    }
}