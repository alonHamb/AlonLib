package org.firstinspires.ftc.teamcode.alonlib.math.geometry

/**
 * A change in pose in 3D space: [dx]/[dy]/[dz] meters of linear motion and [rx]/[ry]/[rz] radians
 * of an axis-angle rotation vector.
 *
 * Unlike [Twist2d], this is *not* wired up to a constant-curvature [Pose3d.exp]/[Pose3d.log] pair
 * -- upstream WPILib implements that SE(3) pose-exponential via a native (JNI) helper with no
 * portable pure-Java reference to port, so [Pose3d.interpolate] instead interpolates translation
 * and rotation independently (lerp + slerp). This type is kept for API parity and for callers
 * that just want to carry `(dx, dy, dz, rx, ry, rz)` around.
 */
data class Twist3d(
    val dx: Double = 0.0,
    val dy: Double = 0.0,
    val dz: Double = 0.0,
    val rx: Double = 0.0,
    val ry: Double = 0.0,
    val rz: Double = 0.0,
) {
    override fun toString() = "Twist3d(dx=$dx, dy=$dy, dz=$dz, rx=$rx, ry=$ry, rz=$rz)"
}
