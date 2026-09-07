package alonlib.math.spline

import alonlib.math.geometry.Pose2d

/** A pose sampled from a [Spline], paired with the spline's curvature at that point. */
data class PoseWithCurvature(val pose: Pose2d = Pose2d.kZero, val curvatureRadPerMeter: Double = 0.0)
