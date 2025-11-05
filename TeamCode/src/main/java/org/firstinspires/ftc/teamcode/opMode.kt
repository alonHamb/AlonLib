package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
@TeleOp(name="Test", group="Test")
class opMode: CommandOpMode() {

    override fun initialize() {
        telemetry.addLine("initialized")
    }

    override fun run() {
        super.run()
        updateTelemetry(telemetry)
    }

}