package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrint
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations

@TeleOp(name = "Blue Main Teleop", group = "Teleop")
class BlueMainTeleop : CommandOpMode() {
    override fun initialize() {
        telemetry.isAutoClear = true
        telemetry.addLine("Robot initializing")
        //RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2)
        telemetry.update()


    }


    override fun run() {
        telemetry.update()
        telemetry.addLine("Robot is running")
        val value = (GamepadEx(gamepad1).leftY * 360).degrees
        robotPrint(" value ${GamepadEx(gamepad1).leftY}  degrees ${value.degrees} radians ${value.radians} rotations ${value.rotations}")
        //robotPrint("${GamepadEx(gamepad1).leftY}")
        super.run()
    }
}