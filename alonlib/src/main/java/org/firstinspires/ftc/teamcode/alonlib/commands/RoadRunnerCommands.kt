package org.firstinspires.ftc.teamcode.alonlib.commands

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.command.Subsystem

/**
 * Wraps a RoadRunner [Action] (e.g. a trajectory built with `MecanumDrive.actionBuilder(...).build()`)
 * as a SolversLib [Command], so it can be scheduled as a default/triggered command or combined with
 * [com.seattlesolvers.solverslib.command.SequentialCommandGroup] etc. instead of only being runnable
 * via `Actions.runBlocking` in a plain [com.qualcomm.robotcore.eventloop.opmode.LinearOpMode].
 *
 * Each [execute] call runs the action once -- every loop, so trajectory following stays accurate
 * -- but the resulting telemetry packet is only actually sent to FTC Dashboard at the rate
 * [DashboardTelemetryThrottle] allows, since sending one every loop is a common cause of loop time
 * blowing up. [isFinished] becomes true once the action reports it's done (`run` returns false).
 */
class ActionCommand(private val action: Action, vararg requirements: Subsystem) : CommandBase() {
    private var finished = false
    private val dashboardThrottle = DashboardTelemetryThrottle()

    init {
        addRequirements(*requirements)
    }

    override fun initialize() {
        finished = false
    }

    override fun execute() {
        val packet = TelemetryPacket()
        finished = !action.run(packet)
        dashboardThrottle.send(packet)
    }

    override fun isFinished(): Boolean = finished
}

/** @see ActionCommand */
fun Action.asCommand(vararg requirements: Subsystem): Command = ActionCommand(this, *requirements)
