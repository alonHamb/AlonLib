package alonlib.hardware.sensors

import alonlib.math.geometry.Quaternion
import alonlib.units.degrees
import com.qualcomm.robotcore.hardware.GyroSensor
import com.qualcomm.robotcore.hardware.Gyroscope
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/** The legacy [GyroSensor] interface -- superseded by [HaIMU] on modern hubs, but still SDK-supported for older gyro modules. */
class HaGyroscope(val gyroSensor: GyroSensor) : HardwareDevice by gyroSensor {

	constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(GyroSensor::class.java, id))

	private val gyroScopeAngularVelocity = (gyroSensor as Gyroscope).getAngularVelocity(AngleUnit.DEGREES)

	fun calibrate() = gyroSensor.calibrate()
	val isCalibrating get() = gyroSensor.isCalibrating

	val heading get() = gyroSensor.heading.degrees
	val rotationFraction get() = gyroSensor.rotationFraction

	val rawX get() = gyroSensor.rawX()
	val rawY get() = gyroSensor.rawY()
	val rawZ get() = gyroSensor.rawZ()

	fun resetZAxisIntegrator() = gyroSensor.resetZAxisIntegrator()

	/**
	 * a [Quaternion] with x y z angular velocities in degrees per second
	 */
	val angularVelocity = Quaternion(0.0, gyroScopeAngularVelocity.xRotationRate.toDouble(), gyroScopeAngularVelocity.yRotationRate.toDouble(), gyroScopeAngularVelocity.zRotationRate.toDouble())

}
