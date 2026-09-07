package alonlib.purepursuit.waypoints

import alonlib.math.geometry.Pose2d
import alonlib.purepursuit.types.WaypointType

/** The first waypoint of every [alonlib.purepursuit.Path] -- the robot never actually traverses it, so it carries no motion settings. */
class StartWaypoint(override val pose: Pose2d) : Waypoint {

	override val type get() = WaypointType.START
	override val followDistance get() = 0.0 // Never used.
	override val timeoutMilliseconds: Long = -1

	override fun toString() = "StartWaypoint(${pose.x}, ${pose.y})"
}
