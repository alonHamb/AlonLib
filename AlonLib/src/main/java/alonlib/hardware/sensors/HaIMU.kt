package alonlib.hardware.sensors

import alonlib.math.geometry.AngularPosition2d
import alonlib.math.geometry.AngularPosition3d
import alonlib.math.geometry.Quaternion
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

/** Wraps the FTC SDK's universal [IMU] interface -- works with any modern hub-mounted IMU, regardless of vendor. */
class HaIMU(val imu: IMU) : HardwareDevice by imu {

	constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(IMU::class.java, id))

	// --- setup ---

	fun initialize(parameters: IMU.Parameters) = imu.initialize(parameters)
	fun resetYaw() = imu.resetYaw()

	// --- readings ---
	private val sdkAngularVelocity get() = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
	private val absoluteImuOrientation get() = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
	private val relativeImuOrientation get() = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)

	/** The current yaw/pitch/roll. */
	val yawPitchRollAngles: AngularPosition3d get() = AngularPosition3d(imu.robotYawPitchRollAngles.getRoll(AngleUnit.DEGREES), imu.robotYawPitchRollAngles.getPitch(AngleUnit.DEGREES), imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))

	/** Yaw as a [AngularPosition2d].*/
	val yaw get() = AngularPosition2d.fromDegrees(imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))

	/**
	 * the imu orientation from an extrinsic perspective
	 */
	val extrinsicOrientation: AngularPosition3d get() = AngularPosition3d(absoluteImuOrientation.firstAngle.toDouble(), absoluteImuOrientation.secondAngle.toDouble(), absoluteImuOrientation.thirdAngle.toDouble())

	/**
	 * the imu orientation from an intrinsic perspective
	 */
	val intrinsicOrientation: AngularPosition3d get() = AngularPosition3d(relativeImuOrientation.thirdAngle.toDouble(), relativeImuOrientation.secondAngle.toDouble(), relativeImuOrientation.thirdAngle.toDouble())

	/**
	 * the imu orientation as a quaternion
	 */
	val orientationAsQuaternion: Quaternion get() = Quaternion(imu.robotOrientationAsQuaternion.w.toDouble(), imu.robotOrientationAsQuaternion.x.toDouble(), imu.robotOrientationAsQuaternion.y.toDouble(), imu.robotOrientationAsQuaternion.z.toDouble())

	/**
	 * a quaternion holding the current angular velocity in degrees per second in all three axis
	 */
	val angularVelocity: Quaternion get() = Quaternion(0.0, sdkAngularVelocity.xRotationRate.toDouble(), sdkAngularVelocity.yRotationRate.toDouble(), sdkAngularVelocity.zRotationRate.toDouble())
}
