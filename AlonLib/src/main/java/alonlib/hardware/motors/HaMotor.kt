package alonlib.hardware.motors

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import alonlib.hardware.Data
import alonlib.hardware.Data.Motors.Direction
import alonlib.hardware.Data.Motors.GoBILDA
import alonlib.hardware.Data.Motors.RunMode
import alonlib.math.PIDFGains
import alonlib.math.control.PIDFController
import alonlib.math.control.SimpleMotorFeedforward
import alonlib.math.geometry.Rotation2d
import alonlib.robotPrintError
import alonlib.units.AngularAcceleration
import alonlib.units.AngularVelocity
import alonlib.units.Current
import alonlib.units.Length
import alonlib.units.Percentage
import alonlib.units.Voltage
import alonlib.units.amps
import alonlib.units.compareTo
import alonlib.units.degrees
import alonlib.units.fraction
import alonlib.units.meters
import alonlib.units.microseconds
import alonlib.units.nanoseconds
import alonlib.units.rotations
import alonlib.units.rpm
import alonlib.units.rpmPerSecond
import alonlib.units.rps
import alonlib.units.volts
import kotlin.math.absoluteValue

/**
 * AlonLib's motor hardware wrapper -- owns an SDK [DcMotorEx] directly (encoder position/velocity
 * read via [hub]'s bulk data, not the SDK's own per-call reads), with its own software PIDF loop
 * for [RunMode.PositionControl]/[RunMode.VelocityControl] and software current limiting.
 *
 * Optional [followers] mirror this motor's [percentOutput] every time it's set (directly, or via
 * [voltage]/[update]) -- construct each one the way you want it to run (direction, zero-power
 * behavior, ...) and pass it in here; they never run their own PID.
 */
class HaMotor(hardwareMap: HardwareMap, id: String, val cpr: Number, val rpm: AngularVelocity, private vararg val followers: HaMotor?) : HardwareDevice {

	constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA, vararg followers: HaMotor) : this(
		hardwareMap,
		id,
		type.cpr,
		type.rpm.rpm,
		*followers
	)

	// --- motor parameters ---
	private val ticksPerRev: Double = cpr.toDouble()

	var distancePerRevolution = 1.meters

	// --- hardware declaration ---
	val hub: LynxModule = hardwareMap.get(LynxModule::class.java, "Control Hub")
	val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, id).apply {
		mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
	}
	private val batteryVoltage: Voltage
		get() = hub.getInputVoltage(VoltageUnit.VOLTS).volts

	val velocityController = PIDFController()
	val positionController = PIDFController()
	var feedForwardController = SimpleMotorFeedforward()

	private val time = System.nanoTime().nanoseconds
	private var lastVelocity = 0.0.rpm
	private var lastTimestamp = 0.nanoseconds
	private var timeStep = time - lastTimestamp

	// --- motor configurations ---

	/**
	sets the behavior of the motor when stop() is called or when you set [percentOutput] to zero
	 */
	var zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		set(value) {
			field = value
			motor.zeroPowerBehavior = value.sdkBehavior
			followers.forEach { it?.zeroPowerBehavior = value }
		}

	/**
	 * the direction the motor to rotates
	 * @param Direction.Forward clockwise
	 * @param Direction.Reverse counterclockwise
	 */
	var runningDirection: Direction
		get() {
			return when (motor.direction) {
				DcMotorSimple.Direction.REVERSE -> Direction.Reverse
				DcMotorSimple.Direction.FORWARD -> Direction.Forward
			}
		}
		set(value) {
			motor.direction = when (value) {
				Direction.Forward -> DcMotorSimple.Direction.FORWARD
				Direction.Reverse -> DcMotorSimple.Direction.REVERSE
			}
			followers.forEach { it?.runningDirection = value }
		}

	/**
	 * the way the [update] function is used to control the motor.
	 * @param RunMode.RawPower doesn't do anything
	 * @param RunMode.PositionControl sends [setPoint] to the pid controller as degrees between [minimumAngle] and [maximumAngle]
	 * @param RunMode.VelocityControl sends [setPoint] to the pid controller as rpm between -[rpm] and [rpm]
	 */
	var runMode: RunMode = RunMode.RawPower
		set(value) {
			field = value
			followers.forEach { it?.runMode = value }
		}

	/**
	 * the way you set how position is
	 */
	var distanceMode: Data.Motors.DistanceMode = Data.Motors.DistanceMode.Angular

	// --- state getters and setters ---

	/**
	 * the smallest number you can send to the motor with the [percentOutput] property
	 */
	var minPercentOutput = (-1).fraction
		set(percentOutput) {
			field = percentOutput.coerceIn((-1).fraction, maxPercentOutput)
		}

	/**
	 * the largest number you can send to the motor with the [percentOutput] property
	 */
	var maxPercentOutput = 1.fraction
		set(percentOutput) {
			field = percentOutput.coerceIn(minPercentOutput, 1.fraction)
		}

	/**
	 * sets the percent output of the motor.
	 * is clamped between properties [minPercentOutput] and [maxPercentOutput],
	 * default is -1.0 and 1.0
	 *
	 * mirrored to every one of [followers] once applied here.
	 */
	var percentOutput: Percentage = 0.fraction
		get() = motor.power.fraction
		set(percentOutput) {
			if (!(forwardLimit() && percentOutput.asFraction > 0.0) && !(reverseLimit() && percentOutput.asFraction < 0.0)) {
				field = percentOutput.coerceIn(minPercentOutput, maxPercentOutput)
				motor.power = field.asFraction
				followers.forEach { it?.percentOutput = percentOutput }
			} else {
				robotPrintError("limit reached")
			}

		}

	/**
	 * the voltage sent to the motor
	 */
	var voltage: Voltage = 0.volts
		get() {
			return batteryVoltage * percentOutput.asFraction
		}
		set(value) {
			field = value
			percentOutput = (value.asVolts / batteryVoltage.asVolts).fraction.coerceIn(minPercentOutput, maxPercentOutput)
			followers.forEach { it?.voltage = value }
		}

	/**
	 * the current level of the motor.
	 * when called returns the current being drawn by the motor
	 * when being set sets the [currentLimit] of the motor
	 */
	val current: Current
		get() {
			return motor.getCurrent(CurrentUnit.AMPS).amps
		}

	/**
	 * the motors current limit

	 * set to 0.0 amps to disable current limiting entirely.
	 */
	var currentLimit: Current = 0.0.amps
		set(value) {
			field = value
			followers.forEach { it?.currentLimit = value }
		}

	/**
	 * Software forward limit, ONLY for [percentOutput] control.
	 */
	var forwardLimit: () -> Boolean = { false }

	/**
	 * Software reverse limit, ONLY for [percentOutput] control.
	 */
	var reverseLimit: () -> Boolean = { false }

	/**
	 * sets the maximum position setpoint you can set to the motor
	 */
	var maximumAngle: Rotation2d = 180.degrees
		set(value) {
			when (value > minimumAngle) {
				true  -> field = value
				false -> robotPrintError("maximum angle smaller then minimum position")
			}
		}

	/**
	 * sets the minimum position setpoint you can set to the motor
	 */
	var minimumAngle: Rotation2d = (-180).degrees
		set(value) {
			when (value < maximumAngle) {
				true  -> field = value
				false -> robotPrintError("minimum angle bigger then maximum position")
			}
		}

	/**
	 * when called gives the current [angle] from the motor encoder
	 *
	 * when set sets the angle [setPoint] of the motor
	 */
	var angle: Rotation2d = 0.degrees
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.portNumber) / ticksPerRev)).rotations
		set(angle) {
			field = angle
			setPoint = angle.degrees.coerceIn(minimumAngle.degrees, maximumAngle.degrees)
			followers.forEach { it?.angle = angle }
		}

	/**
	 * when called gives the current [position] from the motor encoder
	 *
	 * when set sets the position [setPoint] of the motor
	 */
	var position: Length = 0.meters
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.portNumber) / ticksPerRev) * distancePerRevolution.asMeters).meters
		set(position) {
			field = position
			setPoint = position.asMeters.coerceIn(minimumAngle.degrees, maximumAngle.degrees)
			followers.forEach { it?.position = position }
		}

	var minimumPosition: Length = 0.meters
		set(value) {
			when (value > maximumPosition) {
				true  -> field = value
				false -> robotPrintError("minimum position bigger then maximum position")
			}
		}

	var maximumPosition: Length = (distancePerRevolution.asMeters).meters
		set(value) {
			when (value < minimumPosition) {
				true  -> field = value
				false -> robotPrintError("maximum position smaller then minimum position")
			}
		}

	var minimumVelocity: AngularVelocity = -rpm

	var maximumVelocity: AngularVelocity = rpm

	/**
	 * when called gives the current [velocity] from the motor encoder
	 *
	 * when set sets the velocity [setPoint] of the motor
	 */
	var velocity: AngularVelocity = 0.rpm
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorVelocity(motor.portNumber) / ticksPerRev)).rps
		set(velocity) {
			when (velocity) {
				0.rpm -> {
					motor.power = 0.0
				}

				else  -> {
					field = velocity
					setPoint = velocity.coerceIn(minimumVelocity, maximumVelocity).asRpm
				}
			}
			followers.forEach { it?.velocity = velocity }
		}

	// --- pid properties ---

	/**
	 * the [pidfGains] for the motor's PIDF controller used in any closed loop [runMode]
	 */
	var pidfGains: PIDFGains = PIDFGains()
		set(gains) {
			velocityController.gains = gains
			positionController.gains = gains
			feedForwardController = SimpleMotorFeedforward(gains)
			field = gains
			followers.forEach { it?.pidfGains = gains }
		}

	/**
	the current [setPoint] for the motors pid controller

	if the run mode is [RunMode.PositionControl] the unit is degrees.
	if the run mode is [RunMode.VelocityControl] the unit is rpm.

	when [distanceMode] is [Data.Motors.DistanceMode.Linear] the unit is meters
	 */
	var setPoint: Double = 0.0
		set(setPoint) {
			when (positionController.gains.integral > 0 || velocityController.gains.integral > 0) {
				true  -> {
					positionController.reset()
					velocityController.reset()
				}

				false -> {}
			}
			when (this.runMode) {
				RunMode.PositionControl -> {
					when (distanceMode) {
						Data.Motors.DistanceMode.Linear  -> {
							positionController.setPoint =
								setPoint.coerceIn(minimumPosition.asMeters, maximumPosition.asMeters)
							field = setPoint
						}

						Data.Motors.DistanceMode.Angular -> {
							positionController.setPoint =
								setPoint.coerceIn(minimumAngle.degrees, maximumAngle.degrees)
							field = setPoint
						}
					}

				}

				RunMode.VelocityControl -> {
					velocityController.setPoint = setPoint.coerceIn(-rpm.rpm, rpm.rpm)
					field = setPoint
				}

				RunMode.RawPower        -> {}
			}
			followers.forEach { it?.setPoint = setPoint }
		}

	/**
	 *  the current [error] of the pid controller
	 *
	 *  uses the same units as the controllers [setPoint]
	 */
	val error: Double
		get() {
			return when (runMode) {
				RunMode.PositionControl -> {
					positionController.positionError
				}

				RunMode.VelocityControl -> {
					velocityController.positionError
				}

				RunMode.RawPower        -> {
					0.0
				}
			}

		}

	private val positionPidOutputVoltage: Voltage
		get() {
			return when (distanceMode) {
				Data.Motors.DistanceMode.Angular -> (positionController.calculate(angle.degrees) + feedForwardController.calculate(
					velocity.asRpm,
					acceleration.asRpmPerSecond
				)).volts

				Data.Motors.DistanceMode.Linear  -> (positionController.calculate(angle.rotations * distancePerRevolution.asMeters) + feedForwardController.calculate(
					velocity.asRpm, acceleration.asRpmPerSecond
				)).volts
			}
		}

	private val velocityPidOutputVoltage: Voltage
		get() {
			return (velocityController.calculate(velocity.asRpm) + feedForwardController.calculate(velocity.asRpm, acceleration.asRpmPerSecond)).volts
		}

	/**
	 * the tolerance used for the [inTolerance] properties
	 *
	 * uses the units of the pid controllers [setPoint]
	 */
	var tolerance: Double = 0.0
		set(value) {
			when (runMode) {
				RunMode.PositionControl -> positionController.setTolerance(value)
				RunMode.VelocityControl -> velocityController.setTolerance(value)
				RunMode.RawPower        -> {}
			}
			followers.forEach { it?.tolerance = value }
		}

	/**
	 * @returns true if the error of the pid controller is within the tolerance
	 */
	val inTolerance: Boolean
		get() {
			return when (runMode) {
				RunMode.PositionControl -> {
					positionController.inTolerance()
				}

				RunMode.VelocityControl -> {
					velocityController.inTolerance()
				}

				RunMode.RawPower        -> {
					true
				}
			}
		}

	// --- operations functions ---

	/**
	 * called every loop by [update].
	 */
	private fun limitCurrent() {
		if (currentLimit > 0.0.amps)
			if (current.asAmps.absoluteValue > currentLimit.asAmps) {
				// scale back voltage proportionally to how far over the limit we are
				val scale: Double = currentLimit.asAmps / current.asAmps.absoluteValue
				voltage *= scale
			}
	}

	/**
	 * rotations/second^2, estimated from consecutive [velocity] reads across [update] calls -- fed
	 * into [feedForwardController]'s acceleration term.
	 */
	private val acceleration: AngularAcceleration
		get() {
			velocity.asRps
			val acceleration = if (timeStep > 1.microseconds) ((velocity.asRpm - lastVelocity.asRpm) / timeStep.asSeconds).rpmPerSecond else 0.0.rpmPerSecond
			lastVelocity = velocity
			lastTimestamp = time
			return acceleration
		}

	/**
	 * stops the motor
	 *
	 * does the same as setting [percentOutput] to 0.0, and stops every one of [followers] too.
	 */
	fun stop() {
		percentOutput = 0.fraction
		motor.power = 0.0
		followers.forEach { it?.stop() }
	}

	/**
	 * updates the motors pid controller
	 *
	 * must be called every loop
	 */
	fun update() {
		when (this.runMode) {
			RunMode.VelocityControl -> voltage = velocityPidOutputVoltage

			RunMode.PositionControl -> voltage = positionPidOutputVoltage

			RunMode.RawPower        -> {}
		}
		limitCurrent()
		lastTimestamp = time
		followers.forEach { it?.update() }
	}

	// --- hardware device shit ---

	override fun getManufacturer(): HardwareDevice.Manufacturer {
		return HardwareDevice.Manufacturer.Unknown
	}

	override fun getDeviceName(): String {
		return "HaMotor"
	}

	override fun getConnectionInfo(): String {
		return motor.connectionInfo
	}

	override fun getVersion(): Int {
		return motor.version
	}

	override fun resetDeviceConfigurationForOpMode() {
		motor.resetDeviceConfigurationForOpMode()
		followers.forEach { it?.resetDeviceConfigurationForOpMode() }
	}

	override fun close() {
		motor.close()
		followers.forEach { it?.close() }
	}

}
