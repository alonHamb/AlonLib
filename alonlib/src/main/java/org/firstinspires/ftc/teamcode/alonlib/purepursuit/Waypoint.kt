package org.firstinspires.ftc.teamcode.alonlib.purepursuit

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.WaypointType

/** A point a pure-pursuit [Path] traverses. See [org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.GeneralWaypoint] and its subtypes. */
interface Waypoint {
    val type: WaypointType
    val pose: Pose2d
    val followDistance: Double
    val timeoutMilliseconds: Long
}
