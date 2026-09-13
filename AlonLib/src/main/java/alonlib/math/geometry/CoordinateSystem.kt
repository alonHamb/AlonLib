package alonlib.math.geometry

import alonlib.robotPrintError
import kotlin.math.abs

/** Converts [Point3d]/[AngularPosition3d]/[Pose3d]/[Transform3d] between different coordinate system conventions. */
class CoordinateSystem(positiveX: CoordinateAxis, positiveY: CoordinateAxis, positiveZ: CoordinateAxis) {

	/** Rotation from this coordinate system to the NWU coordinate system, applied extrinsically. */
	internal val rotation: AngularPosition3d

	init {
		// Change-of-basis matrix from this coordinate system to NWU: each column is one of this
		// system's basis vectors, expressed in NWU.
		val r = arrayOf(
			doubleArrayOf(positiveX.axis.x, positiveY.axis.x, positiveZ.axis.x),
			doubleArrayOf(positiveX.axis.y, positiveY.axis.y, positiveZ.axis.y),
			doubleArrayOf(positiveX.axis.z, positiveY.axis.z, positiveZ.axis.z),
		)

		val det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
				r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
				r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
		if (abs(det + 1.0) < 1e-9) {
			robotPrintError("CoordinateSystem requires a right-handed system, but a left-handed one was provided")
		}

		rotation = AngularPosition3d.fromRotationMatrix(r)
	}

	companion object {

		private val nwu = CoordinateSystem(CoordinateAxis.N(), CoordinateAxis.W(), CoordinateAxis.U())
		private val edn = CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.D(), CoordinateAxis.N())
		private val ned = CoordinateSystem(CoordinateAxis.N(), CoordinateAxis.E(), CoordinateAxis.D())

		/** North-West-Up: +X north, +Y west, +Z up. */
		fun NWU() = nwu

		/** East-Down-North: +X east, +Y down, +Z north. */
		fun EDN() = edn

		/** North-East-Down: +X north, +Y east, +Z down. */
		fun NED() = ned

		fun convert(translation: Point3d, from: CoordinateSystem, to: CoordinateSystem) =
			translation.rotateBy(from.rotation).rotateBy(to.rotation.inverse())

		fun convert(rotation: AngularPosition3d, from: CoordinateSystem, to: CoordinateSystem) =
			rotation.rotateBy(from.rotation).rotateBy(to.rotation.inverse())

		fun convert(pose: Pose3d, from: CoordinateSystem, to: CoordinateSystem) =
			Pose3d(convert(pose.translation, from, to), convert(pose.rotation, from, to))

		fun convert(transform: Transform3d, from: CoordinateSystem, to: CoordinateSystem): Transform3d {
			val coordRot = from.rotation.rotateBy(to.rotation.inverse())
			return Transform3d(
				convert(transform.translation, from, to),
				coordRot.inverse().rotateBy(transform.rotation.rotateBy(coordRot)),
			)
		}
	}
}
