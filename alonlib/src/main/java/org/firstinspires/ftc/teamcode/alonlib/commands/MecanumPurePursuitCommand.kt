package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.Path
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.Waypoint

/**
 * Drives a mecanum-style robot along a pure-pursuit [Path], every scheduler tick, until it
 * finishes.
 *
 * Takes [driveRobotCentric]/[stop] lambdas rather than a concrete drivebase type -- pass
 * `drive::driveRobotCentric`/`drive::stop` from whatever [org.firstinspires.ftc.teamcode.alonlib.drives]
 * class you're using.
 */
class MecanumPurePursuitCommand(
	private val driveRobotCentric: (strafe: Double, forward: Double, turn: Double) -> Unit,
	private val stop: () -> Unit,
	private val pose: () -> Pose2d,
	vararg waypoints: Waypoint,
) : CommandBase() {

	private val path = Path(*waypoints)

	fun addWaypoint(waypoint: Waypoint) = path.add(waypoint)
	fun addWaypoints(vararg waypoints: Waypoint) = waypoints.forEach { path.add(it) }
	fun removeWaypointAtIndex(index: Int) = path.removeAt(index)

	override fun initialize() = path.init()

	override fun execute() {
		val robotPose = pose()
		val (strafe, forward, turn) = path.loop(robotPose.translation.x, robotPose.translation.y, robotPose.rotation.radians)
		driveRobotCentric(strafe, forward, turn)
	}

	override fun end(interrupted: Boolean) = stop()

	override fun isFinished() = path.isFinished()
}

private operator fun DoubleArray.component1() = this[0]
private operator fun DoubleArray.component2() = this[1]
private operator fun DoubleArray.component3() = this[2]
