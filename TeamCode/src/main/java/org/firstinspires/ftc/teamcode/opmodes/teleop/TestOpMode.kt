package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains
import org.firstinspires.ftc.teamcode.opmodes.teleop.TestOpMode.Constants.kD
import org.firstinspires.ftc.teamcode.opmodes.teleop.TestOpMode.Constants.kF
import org.firstinspires.ftc.teamcode.opmodes.teleop.TestOpMode.Constants.kI
import org.firstinspires.ftc.teamcode.opmodes.teleop.TestOpMode.Constants.kP
import org.firstinspires.ftc.teamcode.opmodes.teleop.TestOpMode.Constants.setPoint
import kotlin.math.round

@TeleOp(name = "Test", group = "Teleop")
@Config
class TestOpMode : CommandOpMode() {
    lateinit var motor: HaMotor
    lateinit var hub: LynxModule
    override fun initialize() {
        motor = HaMotor(hardwareMap, "m3", Motor.GoBILDA.RPM_312).apply {
            pidfGains
            runMode = Motor.RunMode.VelocityControl
            zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        }
        hub = hardwareMap.get(LynxModule::class.java, "Control Hub").apply {
            bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Robot initializing")
        telemetry.update()


    }

    object Constants {
        var kP: Double = 0.0725
        var kI: Double = 0.0
        var kD: Double = 0.0
        var kF: Double = 0.51
        var setPoint: Double = 312.0 / 2.0
    }

    override fun run() {
        hub.clearBulkCache()
        motor.pidfGains = PIDFGains(kP, kI, kD, kF)
        motor.setPoint = setPoint
        motor.update()
        telemetry.addData("cycle time micro seconds", round(motor.positonController.period * 1000))
        telemetry.addData("motor error", motor.error)
        telemetry.addData("motor position", motor.position.degrees)
        telemetry.addData("motor velocity", motor.velocity.asRpm)
        telemetry.addData("motor setpoint", motor.setPoint)
        telemetry.addData("motor power", motor.percentOutput)
        telemetry.addData("motor voltage", motor.voltage)
        telemetry.addData("motor pid", motor.pidfGains)
        super.run()
        telemetry.update()
    }
}
