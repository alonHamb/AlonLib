package alonlib.math.geometry

// Convenience extension-function bridges from the 2D geometry types to their 3D counterparts,
// complementing the `Xyz3d(xyz2d)` constructors those 3D types already expose.

fun AngularPosition2d.toRotation3d() = AngularPosition3d(this)
fun Point2d.toTranslation3d() = Point3d(this)
fun Transform2d.toTransform3d() = Transform3d(this)
fun Pose2d.toPose3d() = Pose3d(this)
