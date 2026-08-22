package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

/**
 * A [Double]-keyed, [Double]-valued [InterpolatingTreeMap], for the common case of a lookup table
 * built up at runtime (e.g. shooter RPM by measured distance).
 */
class InterpolatingDoubleTreeMap : InterpolatingTreeMap<Double, Double>(
    InverseInterpolator.forDouble,
    Interpolator.forDouble,
)
