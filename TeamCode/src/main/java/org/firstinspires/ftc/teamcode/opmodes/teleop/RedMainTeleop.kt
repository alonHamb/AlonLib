package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.RobotContainer
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance

@TeleOp(name = "Red Main Teleop", group = "Teleop")
class RedMainTeleop : CommandOpMode() {
    override fun initialize() {
        val alliance = Alliance.Red
        FtcDashboard.getInstance().telemetry.addLine("Robot initializing")
        RobotContainer(hardwareMap, FtcDashboard.getInstance().telemetry, gamepad1, gamepad2, alliance)
        FtcDashboard.getInstance().telemetry.update()
        updateTelemetry(telemetry)


    }

    override fun run() {
        FtcDashboard.getInstance().telemetry.addLine("Robot is running")
        super.run()
        FtcDashboard.getInstance().telemetry.update()
        updateTelemetry(telemetry)
    }
}