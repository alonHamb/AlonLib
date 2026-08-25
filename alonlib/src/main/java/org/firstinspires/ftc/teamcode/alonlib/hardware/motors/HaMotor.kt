package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.Direction
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.GoBILDA
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.RunMode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.ZeroPowerBehavior.FLOAT
import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.Current
import org.firstinspires.ftc.teamcode.alonlib.units.Percentage
import org.firstinspires.ftc.teamcode.alonlib.units.Voltage
import org.firstinspires.ftc.teamcode.alonlib.units.amps
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.alonlib.units.rps
import org.firstinspires.ftc.teamcode.alonlib.units.volts
import kotlin.math.absoluteValue
import kotlin.math.sign

/**
 * AlonLib's motor hardware wrapper -- owns an SDK [DcMotorEx] directly (encoder position/velocity
 * read via [hub]'s bulk data, not the SDK's own per-call reads), with its own software PIDF loop
 * for [RunMode.POSITION_CONTROL]/[RunMode.VELOCITY_CONTROL] and software current limiting.
 *
 * Optional [followers] mirror this motor's [percentOutput] every time it's set (directly, or via
 * [voltage]/[update]) -- construct each one the way you want it to run (direction, zero-power
 * behavior, ...) and pass it in here; they never run their own PID.
 */
class HaMotor(hardwareMap: HardwareMap, id: String, val cpr: Number, val rpm: AngularVelocity, private vararg val followers: HaMotor) : HardwareDevice {
    constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA, vararg followers: HaMotor) : this(
        hardwareMap,
        id,
        type.cpr,
        type.rpm.rpm,
        *followers
    )

    // --- motor parameters ---
    private val ticksPerRev: Double = cpr.toDouble()


    // --- hardware declaration ---
    val hub: LynxModule = hardwareMap.get(LynxModule::class.java, "Control Hub")
    val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, id).apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    private val batteryVoltage: Voltage
        get() = hub.getInputVoltage(VoltageUnit.VOLTS).volts

    val velocityController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    val positionController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    var feedForwardController = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private var lastVelocityRotationsPerSecond = 0.0
    private var lastAccelerationTimestamp = System.nanoTime() / 1e9

    // --- motor configurations ---


    /**
    sets the behavior of the motor when stop() is called or when you set [percentOutput] to zero
     */
    var zeroPowerBehavior = FLOAT
        set(value) {
            field = value
            motor.zeroPowerBehavior = value.sdkBehavior
            followers.forEach { it.zeroPowerBehavior = value }
        }


    /**
     * the direction the motor to rotates
     * @param Direction.FORWARD clockwise
     * @param Direction.REVERSE counterclockwise
     */
    var runningDirection: Direction
        get() {
            return when (motor.direction) {
                DcMotorSimple.Direction.REVERSE -> Direction.REVERSE
                else                            -> Direction.FORWARD
            }
        }
        set(value) {
            motor.direction = when (value) {
                Direction.FORWARD -> DcMotorSimple.Direction.FORWARD
                Direction.REVERSE -> DcMotorSimple.Direction.REVERSE
            }
            followers.forEach { it.runningDirection = value }
        }

    /**
     * the way the [update] function is used to control the motor.
     * @param RunMode.RAW_POWER doesn't do anything
     * @param RunMode.POSITION_CONTROL sends [setPoint] to the pid controller as degrees between [minimumPosition] and [maximumPosition]
     * @param RunMode.VELOCITY_CONTROL sends [setPoint] to the pid controller as rpm between -[rpm] and [rpm]
     */
    var runMode: RunMode = RunMode.RAW_POWER
        set(value) {
            field = value
            followers.forEach { it.runMode = value }
        }


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
                followers.forEach { it.percentOutput = percentOutput }
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
            followers.forEach { it.voltage = value }
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
            followers.forEach { it.currentLimit = value }
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
    var maximumPosition: Rotation2d = 180.degrees
        set(value) {
            when (value > minimumPosition) {
                true  -> field = value
                false -> robotPrintError("maximum position smaller then minimum position")
            }
        }


    /**
     * sets the minimum position setpoint you can set to the motor
     */
    var minimumPosition: Rotation2d = (-180).degrees
        set(value) {
            when (value < maximumPosition) {
                true  -> field = value
                false -> robotPrintError("minimum position bigger then maximum position")
            }
        }

    /**
     * when called gives the current [position] from the motor encoder
     *
     * when set sets the position [setPoint] of the motor
     */
    var position: Rotation2d = 0.degrees
        get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.portNumber) / ticksPerRev)).rotations
        set(position) {
            field = position
            setPoint = position.degrees.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
            followers.forEach { it.position = position }
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
            followers.forEach { it.velocity = velocity }
        }


    // --- pid properties ---

    /**
     * the [pidfGains] for the motor's PIDF controller used in any closed loop [runMode]
     */
    var pidfGains: PIDFGains = PIDFGains()
        set(gains) {
            velocityController.setCoefficients(PIDFCoefficients(gains.kP, gains.kI, gains.kD, 0.0))
            positionController.setCoefficients(PIDFCoefficients(gains.kP, gains.kI, gains.kD, 0.0))
            feedForwardController = SimpleMotorFeedforward(gains.kS, gains.KV, gains.Ka)
            field = gains
            followers.forEach { it.pidfGains = gains }
        }

    /**
    the current [setPoint] for the motors pid controller

    if the run mode is [RunMode.POSITION_CONTROL] the unit is degrees if the run mode is [RunMode.VELOCITY_CONTROL] the unit is rpm
     */
    var setPoint: Double = 0.0
        set(setPoint) {
            when (positionController.i > 0 || velocityController.i > 0) {
                true  -> {
                    positionController.reset()
                    velocityController.reset()
                }

                false -> {}
            }
            when (this.runMode) {
                RunMode.POSITION_CONTROL -> {
                    positionController.setPoint =
                        setPoint.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
                    field = setPoint
                }


                RunMode.VELOCITY_CONTROL -> {
                    velocityController.setPoint = setPoint.coerceIn(-rpm.rpm, rpm.rpm)
                    field = setPoint
                }

                RunMode.RAW_POWER        -> {}
            }
            followers.forEach { it.setPoint = setPoint }
        }

    /**
     *  the current [error] of the pid controller
     *
     *  uses the same units as the controllers [setPoint]
     */
    val error: Double
        get() {
            return when (runMode) {
                RunMode.POSITION_CONTROL -> {
                    positionController.positionError
                }

                RunMode.VELOCITY_CONTROL -> {
                    velocityController.positionError
                }

                RunMode.RAW_POWER        -> {
                    0.0
                }
            }

        }

    /**
     * the tolerance used for the [inTolerance] properties
     *
     * uses the units of the pid controllers [setPoint]
     */
    var tolerance: Double = 0.0
        set(value) {
            when (runMode) {
                RunMode.POSITION_CONTROL -> positionController.setTolerance(value)
                RunMode.VELOCITY_CONTROL -> velocityController.setTolerance(value)
                RunMode.RAW_POWER        -> {}
            }
            followers.forEach { it.tolerance = value }
        }

    /**
     * @returns true if the error of the pid controller is within the tolerance
     */
    val inTolerance: Boolean
        get() {
            return when (runMode) {
                RunMode.POSITION_CONTROL -> {
                    positionController.atSetPoint()
                }

                RunMode.VELOCITY_CONTROL -> {
                    velocityController.atSetPoint()
                }

                RunMode.RAW_POWER        -> {
                    true
                }
            }
        }

    // --- operations functions ---

    /**
     * called every loop by [update].
     */
    private fun limitCurrent() {
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
    private fun estimateAcceleration(): Double {
        val now = System.nanoTime() / 1e9
        val dt = now - lastAccelerationTimestamp
        val velocityRotationsPerSecond = velocity.asRpm / 60.0
        val acceleration = if (dt > 1e-4) (velocityRotationsPerSecond - lastVelocityRotationsPerSecond) / dt else 0.0
        lastVelocityRotationsPerSecond = velocityRotationsPerSecond
        lastAccelerationTimestamp = now
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
        followers.forEach { it.stop() }
    }

    /**
     * updates the motors pid controller
     *
     * must be called every loop
     */
    fun update() {
        when (this.runMode) {
            RunMode.VELOCITY_CONTROL -> voltage =
                (velocityController.calculate(velocity.asRpm) + feedForwardController.calculate(
                    velocity.asRpm,
                    estimateAcceleration()
                ) + pidfGains.kFF * error.sign).volts

            RunMode.POSITION_CONTROL -> voltage =
                (positionController.calculate(position.degrees) + feedForwardController.calculate(
                    velocity.asRpm,
                    estimateAcceleration()
                ) + pidfGains.kFF * error.sign).volts

            RunMode.RAW_POWER        -> {}
        }
        limitCurrent()
        followers.forEach { it.update() }
    }

    // --- hardware device shit ---

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String {
        return "HaMotor"
    }

    override fun getConnectionInfo(): String {
        return ""
    }

    override fun getVersion(): Int {
        return 1
    }

    override fun resetDeviceConfigurationForOpMode() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        followers.forEach { it.resetDeviceConfigurationForOpMode() }
    }

    override fun close() {
        motor.close()
        followers.forEach { it.close() }
    }


}
