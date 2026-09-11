package alonlib.hardware.servos

import alonlib.hardware.Data.Servos.Mode
import alonlib.hardware.Data.Servos.Type
import alonlib.math.geometry.AngularPositon
import alonlib.math.mapRange
import alonlib.robotPrintError
import alonlib.units.AngularVelocity
import alonlib.units.Percentage
import alonlib.units.degrees
import alonlib.units.fraction
import alonlib.units.normalizedDegrees
import alonlib.units.percent
import alonlib.units.rpm
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

/**
 * Optional [followers] mirror this servo's raw `[0, 1]` position every time it's written (via
 * [percentOutput]/[position]/[velocity]) -- construct each one the way you want it to run and pass
 * it in here.
 */
class HaServo(
	hardwareMap: HardwareMap,
	id: String,
	val mode: Mode,
	val type: Type,
	private vararg val followers: HaServo,
) : HardwareDevice {

	// --- servo object declaration ---

	var servo: Servo = hardwareMap.get(Servo::class.java, id)

	init {
		(servo as ServoImplEx).apply {
			pwmRange = when (mode) {
				Mode.Cr        -> PwmControl.PwmRange(type.crPwmRange.first.asMicroseconds, type.crPwmRange.second.asMicroseconds)
				Mode.FullRange -> PwmControl.PwmRange(type.fullRangePwmRange.first.asMicroseconds, type.fullRangePwmRange.second.asMicroseconds)
			}
		}
	}
	// --- state getters and setters ---

	/**
	 * software forward limit only
	 */
	var forwardLimit = { false }

	/**
	 * software reverse limit only
	 */
	var reverseLimit = { false }

	/**
	 * the maximum [percentOutput] to be sent to the servo when in [Mode.Cr]
	 */
	var maxPercentOutput = 1.0.fraction
		set(value) {
			field = value.coerceIn(minPercentOutput..maxPercentOutput)
		}

	/**
	 * the minimum [percentOutput] to be sent to the servo when in [Mode.Cr]
	 */
	var minPercentOutput = 0.0.fraction
		set(value) {
			field = value.coerceIn(minPercentOutput..maxPercentOutput)

		}

	/**
	 * a way to control the output to the servo as a [Percentage]
	 */
	var percentOutput: Percentage = 0.0.fraction
		set(value) {
			if (!(forwardLimit() && value.asFraction > 0) && !(reverseLimit() && value.asFraction < 0)) {
				field = value
				servo.position = value.coerceIn(minPercentOutput..maxPercentOutput).asFraction
			} else {
				robotPrintError("limit reached")
			}
		}
		get() = servo.position.fraction

	/**
	 * half of [Type.range], in degrees -- the most a [AngularPositon] can represent [position] as an
	 * offset from the center of the servo's sweep without exceeding the (-180, 180] domain that
	 * Rotation2d normalizes into (350deg/2 = 175deg, the widest built-in [Type]). [position],
	 * [minPosition], and [maxPosition] are all relative to this center, i.e. 0 degrees is the
	 * middle of the servo's travel, not one end of it.
	 */
	private val halfRange = type.range / 2.0

	/**
	 * the maximum [position] to be sent to the servo
	 */
	var maxPosition: AngularPositon = halfRange
		set(value) {
			field = value.degrees.coerceIn(-halfRange.normalizedDegrees..halfRange.normalizedDegrees).degrees
		}

	/**
	 * the minimum [position] to be sent to the servo
	 */
	var minPosition: AngularPositon = (-halfRange)
		set(value) {
			field = value.degrees.coerceIn(-halfRange.normalizedDegrees..maxPosition.normalizedDegrees).degrees
		}

	/**
	 * when called returns the last [position] that have been sent to the servo
	 *
	 * when set sets the [position] you want the servo to go to
	 */
	var position: AngularPositon = 0.0.degrees
		get() {
			return mapRange(
				servo.position,
				0.0,
				1.0,
				0.0,
				type.range.degrees
			).degrees
		}
		set(position) {
			when (mode) {
				Mode.Cr        -> robotPrintError("cannot set position in CR mode")
				Mode.FullRange -> {
					field = position
					servo.position = mapRange(
						position.coerceIn(minPosition, maxPosition).degrees,
						0.0,
						type.range.degrees,
						0.0,
						1.0
					)
					followers.forEach { it.position = position }
				}
			}
		}

	/**
	 * the maximum [velocity] to be sent to the servo
	 */
	var maxVelocity: AngularVelocity = type.maxSpeed
		set(value) {
			field = value.asRpm.coerceIn(0.0..type.maxSpeed.asRpm).rpm
			followers.forEach { it.maxVelocity = value }
		}

	/**
	 * the minimum [velocity] to be sent to the servo
	 */
	var minVelocity: AngularVelocity = 0.0.rpm
		set(value) {
			field = value.asRpm.coerceIn(0.0..maxVelocity.asRpm).rpm
			followers.forEach { it.minVelocity = value }
		}

	/**
	 * when called returns the last [velocity] that have been sent to the servo
	 *
	 * when set sets the [velocity] you want the servo to get to
	 */
	var velocity: AngularVelocity = 0.0.rpm
		set(value) {
			when (mode) {
				Mode.Cr        -> {
					servo.position = mapRange(value.coerceIn(minVelocity, maxVelocity).asRpm, 0.0, type.maxSpeed.asRpm, 0.0, 1.0)
					field = value
					followers.forEach { it.velocity = value }
				}

				Mode.FullRange -> robotPrintError("cannot set velocity in full range mode")

			}
		}

	fun stop() {
		when (mode) {
			Mode.Cr        -> percentOutput = 0.0.percent
			Mode.FullRange -> {}
		}
		followers.forEach { it.stop() }
	}

	override fun getManufacturer(): HardwareDevice.Manufacturer {
		return servo.manufacturer
	}

	override fun getDeviceName(): String {
		return servo.deviceName
	}

	override fun getConnectionInfo(): String {
		return servo.connectionInfo
	}

	override fun getVersion(): Int {
		return servo.version
	}

	override fun resetDeviceConfigurationForOpMode() {
		servo.resetDeviceConfigurationForOpMode()
		followers.forEach { it.resetDeviceConfigurationForOpMode() }
	}

	override fun close() {
		servo.close()
		followers.forEach { it.close() }
	}

}
