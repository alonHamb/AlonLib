package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.RobotContainer
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {
        lateinit var robot : RobotContainer
    override fun initialize() {

        val alliance = Alliance.Blue
        val telemetryLevel = TelemetryLevel.Testing
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Robot initializing")
        RobotContainer(
            hardwareMap,
            telemetry,
            gamepad1,
            gamepad2,
            alliance,
            telemetryLevel
        )
        telemetry.update()


    }

    override fun run() {
        super.run()
        telemetry.update()
    }
}
