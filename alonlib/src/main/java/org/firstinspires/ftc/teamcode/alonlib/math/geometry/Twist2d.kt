package org.firstinspires.ftc.teamcode.alonlib.math.geometry

/**
 * A small, planar movement along a constant-curvature arc: [dx]/[dy] meters forward/sideways and
 * [dtheta] radians of rotation.
 *
 * This is what [Pose2d.exp]/[Pose2d.log] integrate to/from -- odometry accumulates one of these
 * per loop instead of naively adding `(dx, dy, dtheta)` straight onto the pose, which is what
 * keeps odometry accurate through a turn instead of just at the sampled instants.
 */
data class Twist2d(val dx: Double = 0.0, val dy: Double = 0.0, val dtheta: Double = 0.0) {
    override fun toString() = "Twist2d(dx=$dx, dy=$dy, dtheta=$dtheta)"
}
