package alonlib.purepursuit.waypoints

import alonlib.math.geometry.Pose2d
import alonlib.purepursuit.types.WaypointType

/** A point a pure-pursuit [alonlib.purepursuit.Path] traverses. See [GeneralWaypoint] and its subtypes. */
interface Waypoint {

	val type: WaypointType
	val pose: Pose2d
	val followDistance: Double
	val timeoutMilliseconds: Long
}
