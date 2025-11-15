package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.RobotContainer

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {
    override fun initialize() {
        FtcDashboard.getInstance().telemetry.addLine("Robot initializing")
        RobotContainer(hardwareMap, FtcDashboard.getInstance().telemetry, gamepad1, gamepad2)
        FtcDashboard.getInstance().telemetry.update()


    }

    override fun run() {
        FtcDashboard.getInstance().telemetry.addLine("Robot is running")
        super.run()
        FtcDashboard.getInstance().telemetry.update()
    }
}