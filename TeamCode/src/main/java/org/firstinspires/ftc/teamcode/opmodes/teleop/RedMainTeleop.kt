package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.RobotContainer
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance

@TeleOp(name = "Red Main Teleop", group = "Teleop")
class RedMainTeleop : CommandOpMode() {
    lateinit var motor = HaDcMotor(hardwareMap, "motor", 8192.0, 435.0)
    override fun initialize() {
        val alliance = Alliance.Red
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Robot initializing")
        RobotContainer(hardwareMap, FtcDashboard.getInstance().telemetry, gamepad1, gamepad2, alliance)
        telemetry.update()


    }

    override fun run() {
        telemetry.addLine("Robot is running")
        super.run()
        telemetry.update()
    }
}