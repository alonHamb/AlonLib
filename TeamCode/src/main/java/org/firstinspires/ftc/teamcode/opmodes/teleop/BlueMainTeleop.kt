package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.RobotContainer

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {
    override fun initialize() {
        telemetry.isAutoClear = true
        telemetry.addLine("Robot initializing")
        RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2)
        telemetry.update()
    }


    override fun run() {
        telemetry.update()
        telemetry.addLine("Robot is running")
        super.run()
    }
}