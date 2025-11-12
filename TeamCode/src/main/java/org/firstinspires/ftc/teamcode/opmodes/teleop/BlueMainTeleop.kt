package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {
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

    }

    fun initSubsystems() {
        exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry)
    }

    fun setDefaultCommands() {

    }

    override fun run() {
        telemetry.update()
        telemetry.addLine("Robot is running")
        super.run()
    }
}