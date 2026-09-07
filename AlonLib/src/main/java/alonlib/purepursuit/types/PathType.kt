package alonlib.purepursuit.types

/** How a [alonlib.purepursuit.Path] picks the "best" intersection when several are found at once. */
enum class PathType {
    HEADING_CONTROLLED,
    WAYPOINT_ORDERING_CONTROLLED,
}
