package org.firstinspires.ftc.teamcode.alonlib.math.geometry

// Convenience extension-function bridges from the 2D geometry types to their 3D counterparts,
// complementing the `Xyz3d(xyz2d)` constructors those 3D types already expose.

fun Rotation2d.toRotation3d() = Rotation3d(this)
fun Translation2d.toTranslation3d() = Translation3d(this)
fun Transform2d.toTransform3d() = Transform3d(this)
fun Pose2d.toPose3d() = Pose3d(this)
