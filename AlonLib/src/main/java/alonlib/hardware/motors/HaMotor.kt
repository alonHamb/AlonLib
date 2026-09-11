package alonlib.hardware.motors

import alonlib.hardware.Data
import alonlib.hardware.Data.Motors.Direction
import alonlib.hardware.Data.Motors.GoBILDA
import alonlib.hardware.Data.Motors.RunMode
import alonlib.math.PIDFGains
import alonlib.math.control.PIDFController
import alonlib.math.control.SimpleMotorFeedforward
import alonlib.math.geometry.AngularPositon
import alonlib.robotPrintError
import alonlib.units.AngularAcceleration
import alonlib.units.AngularVelocity
import alonlib.units.Current
import alonlib.units.Distance
import alonlib.units.LinearVelocity
import alonlib.units.Percentage
import alonlib.units.Voltage
import alonlib.units.amps
import alonlib.units.compareTo
import alonlib.units.degrees
import alonlib.units.fraction
import alonlib.units.meters
import alonlib.units.metersPerSecond
import alonlib.units.microseconds
import alonlib.units.nanoseconds
import alonlib.units.rotations
import alonlib.units.rpm
import alonlib.units.rpmPerSecond
import alonlib.units.rps
import alonlib.units.volts
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import kotlin.math.absoluteValue

/**
 * AlonLib's motor hardware wrapper
 *
 * Optional [followers] mirror this motor every time a property is set
 *
 * construct each one the way you want it to run and pass it in here.
 */
class HaMotor(hardwareMap: HardwareMap, id: String, val ticksPerRev: Number, val rpm: AngularVelocity, private vararg val followers: HaMotor?) : HardwareDevice {

	constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA, vararg followers: HaMotor) : this(
		hardwareMap,
		id,
		type.cpr,
		type.rpm.rpm,
		*followers
	)

	// --- motor parameters ---
	/**
	 * the distance moved for every one rotation of the motor
	 */
	var distancePerRevolution: () -> Distance = { 0.meters }

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
	sets the behavior of the motor when stop() is called or when you set [percentOutput] or [voltage] to zero
	 */
	var zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		get() {
			return when (motor.zeroPowerBehavior) {
				DcMotor.ZeroPowerBehavior.FLOAT   -> Data.Motors.ZeroPowerBehavior.Float
				DcMotor.ZeroPowerBehavior.BRAKE   -> Data.Motors.ZeroPowerBehavior.Brake
				DcMotor.ZeroPowerBehavior.UNKNOWN -> Data.Motors.ZeroPowerBehavior.Unknown
			}
		}
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
	 * @param RunMode.PositionControl sends [setPoint] to the pid controller as degrees or meters, depends on [positionMode]
	 * @param RunMode.VelocityControl sends [setPoint] to the pid controller as rpm
	 */
	var runMode: RunMode = RunMode.RawPower
		set(value) {
			field = value
			followers.forEach { it?.runMode = value }
		}

	/**
	 * determines the unit used when sending setpoint, error and position values are sent to the pid controller
	 */
	var positionMode: Data.Motors.PositionMode = Data.Motors.PositionMode.Angular

	// --- motor limits ---

	/**
	 * the minimum [percentOutput] you can send to the motor
	 */
	var minPercentOutput = (-1).fraction
		set(percentOutput) {
			field = percentOutput.coerceIn(minPercentOutput, maxPercentOutput)
		}

	/**
	 * the maximum [percentOutput] you can send to the motor
	 */
	var maxPercentOutput = 1.fraction
		set(percentOutput) {
			field = percentOutput.coerceIn(minPercentOutput, maxPercentOutput)
		}

	/**
	 * the minimum [voltage] you can send to the motor
	 */
	var minVoltage: Voltage = (-15).volts
		set(voltage) {
			field = voltage.coerceIn(minVoltage, maxVoltage)
		}

	/**
	 * the maximum [voltage] you can send to the motor
	 */
	var maxVoltage: Voltage = 15.volts
		set(voltage) {
			field = voltage.coerceIn(minVoltage, maxVoltage)
		}

	/**
	 * sets the minimum angular [linearPosition] setpoint you can send to the motor
	 */
	var minimumAngle: AngularPositon = (-180).degrees
		set(value) {
			field = value.coerceIn(minimumAngle, maximumAngle)
		}

	/**
	 * sets the maximum angular [linearPosition] setpoint you can send to the motor
	 */
	var maximumAngle: AngularPositon = 360.degrees
		set(value) {
			when (value > minimumAngle) {
				true  -> field = value.coerceIn(minimumAngle, maximumAngle)
				false -> robotPrintError("maximum angle smaller then minimum angle")
			}
		}

	/**
	 * sets the minimum linear [linearPosition] that you can send to the motor
	 */
	var minimumPosition: Distance = 0.meters
		set(value) {
			field = value.coerceIn(minimumPosition, maximumPosition)
		}

	/**
	 * sets the maximum linear [linearPosition] that you can send to the motor
	 */
	var maximumPosition: Distance = (distancePerRevolution().asMeters).meters
		set(value) {
			field = value.coerceIn(minimumPosition, maximumPosition)
		}

	/**
	 * the minimum [angularVelocity] to be sent to the motor
	 */
	var minimumAngularVelocity: AngularVelocity = -rpm
		set(angularVelocity) {
			field = angularVelocity.coerceIn(minimumAngularVelocity, maximumAngularVelocity)
			followers.forEach { it?.minimumAngularVelocity = angularVelocity }
		}

	/**
	 * the maximum [angularVelocity] to be sent to the motor
	 */
	var maximumAngularVelocity: AngularVelocity = rpm
		set(angularVelocity) {
			field = angularVelocity.coerceIn(minimumAngularVelocity, maximumAngularVelocity)
			followers.forEach { it?.maximumAngularVelocity = angularVelocity }
		}

	var minimumLinearVelocity: LinearVelocity = 0.metersPerSecond
		set(linearVelocity) {
			field = linearVelocity.coerceIn(minimumLinearVelocity, maximumLinearVelocity)
			followers.forEach { it?.minimumLinearVelocity = linearVelocity }
		}

	var maximumLinearVelocity: LinearVelocity = (rpm.asRps * distancePerRevolution().asMeters).metersPerSecond
		set(linearVelocity) {
			field = linearVelocity.coerceIn(minimumLinearVelocity, maximumLinearVelocity)
			followers.forEach { it?.maximumLinearVelocity = linearVelocity }
		}

	/**
	 * the motors [current] limit
	 * set to 0.0 amps to disable current limiting entirely.
	 */
	var maxCurrent: Current = 0.0.amps
		set(value) {
			field = value.coerceIn(0.0.amps, 9.amps)
			followers.forEach { it?.maxCurrent = value }
		}

	/**
	 * Software forward limit
	 */
	var forwardLimit: () -> Boolean = { false }

	/**
	 * Software reverse limit
	 */
	var reverseLimit: () -> Boolean = { false }

	// --- state getters and setters ---

	/**
	 * sets the [percentOutput] sent to the motor.
	 * is clamped between [minPercentOutput] and [maxPercentOutput],
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
	 * sets the [voltage] sent to the motor
	 * is clamped between [minVoltage] and [maxVoltage]
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
	 * the [current] being drawn by the motor.
	 */
	val current: Current
		get() {
			return motor.getCurrent(CurrentUnit.AMPS).amps
		}

	/**
	 * when called gives the current [angularPosition] from the motor encoder
	 *
	 * when set sets the [setPoint] of the motor with [degrees]
	 */
	var angularPosition: AngularPositon
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.portNumber) / ticksPerRev.toDouble())).rotations
		set(angle) {
			setPoint = angle.degrees.coerceIn(minimumAngle.degrees, maximumAngle.degrees)
			followers.forEach { it?.angularPosition = angle }
		}

	/**
	 * when called gives the current [linearPosition] from the motor encoder
	 *
	 * when set sets the [setPoint] of the motor with [meters]
	 */
	var linearPosition: Distance
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.portNumber) / ticksPerRev.toDouble()) * distancePerRevolution().asMeters).meters
		set(position) {
			setPoint = position.asMeters.coerceIn(minimumAngle.degrees, maximumAngle.degrees)
			followers.forEach { it?.linearPosition = position }
		}

	/**
	 * when called gives the current [angularVelocity] from the motor encoder
	 *
	 * when set sets the [setPoint] of the motor with [rpm]
	 */
	var angularVelocity: AngularVelocity
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorVelocity(motor.portNumber) / ticksPerRev.toDouble())).rps
		set(velocity) {
			when (velocity) {
				0.rpm -> {
					motor.power = 0.0
				}

				else  -> {
					setPoint = velocity.coerceIn(minimumAngularVelocity, maximumAngularVelocity).asRpm
				}
			}
			followers.forEach { it?.angularVelocity = velocity }
		}

	/**
	 * when called gives the current [linearVelocity] from the motor encoder
	 *
	 * when set sets the [setPoint] of the motor with [metersPerSecond]
	 */
	var linearVelocity: LinearVelocity
		get() = (runningDirection.multiplier * (hub.bulkData.getMotorVelocity(motor.portNumber) / ticksPerRev.toDouble()) * distancePerRevolution().asMeters).metersPerSecond
		set(velocity) {
			when (velocity) {
				0.metersPerSecond -> {
					motor.power = 0.0
				}

				else              -> {
					setPoint = velocity.coerceIn(minimumLinearVelocity, maximumLinearVelocity).asMetersPerSecond
				}
			}
			followers.forEach { it?.linearVelocity = velocity }
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

	if the run mode is [RunMode.PositionControl] the unit depends on [Data.Motors.PositionMode],
	[meters] for linear and [degrees] for angular.
	if the run mode is [RunMode.VelocityControl] the unit is [rpm].
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
					when (positionMode) {
						Data.Motors.PositionMode.Linear  -> {
							positionController.setPoint =
								setPoint.coerceIn(minimumPosition.asMeters, maximumPosition.asMeters)
							field = setPoint
						}

						Data.Motors.PositionMode.Angular -> {
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
			return when (positionMode) {
				Data.Motors.PositionMode.Angular -> (positionController.calculate(angularPosition.degrees) + feedForwardController.calculate(
					angularVelocity.asRpm,
					acceleration.asRpmPerSecond
				)).volts

				Data.Motors.PositionMode.Linear  -> (positionController.calculate(angularPosition.rotations * distancePerRevolution().asMeters) + feedForwardController.calculate(
					angularVelocity.asRpm, acceleration.asRpmPerSecond
				)).volts
			}
		}

	private val velocityPidOutputVoltage: Voltage
		get() {
			return (velocityController.calculate(angularVelocity.asRpm) + feedForwardController.calculate(angularVelocity.asRpm, acceleration.asRpmPerSecond)).volts
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
	 * @returns whether the motor is in the [tolerance] window
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
		if (maxCurrent > 0.0.amps)
			if (current.asAmps.absoluteValue > maxCurrent.asAmps) {
				// scale back voltage proportionally to how far over the limit we are
				val scale: Double = maxCurrent.asAmps / current.asAmps.absoluteValue
				voltage *= scale
			}
	}

	/**
	 * rotations/second^2, estimated from consecutive [angularVelocity] reads across [update] calls -- fed
	 * into [feedForwardController]'s acceleration term.
	 */
	private val acceleration: AngularAcceleration
		get() {
			angularVelocity.asRps
			val acceleration = if (timeStep > 1.microseconds) ((angularVelocity.asRpm - lastVelocity.asRpm) / timeStep.asSeconds).rpmPerSecond else 0.0.rpmPerSecond
			lastVelocity = angularVelocity
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
