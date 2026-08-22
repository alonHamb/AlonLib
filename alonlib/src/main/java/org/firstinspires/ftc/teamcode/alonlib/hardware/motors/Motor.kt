package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.alonlib.hardware.HardwareDevice
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward

/**
 * A rich wrapper over the FTC SDK's [DcMotor]: encoder distance/velocity tracking, and optional
 * closed-loop velocity/position control layered on top of the SDK's raw power API.
 */
open class Motor(val motor: DcMotor, protected val type: GoBILDA = GoBILDA.NONE) : HardwareDevice {

    constructor(hardwareMap: HardwareMap, id: String, gobildaType: GoBILDA = GoBILDA.NONE) :
            this(hardwareMap.get(DcMotor::class.java, id), gobildaType)

    /** For a motor whose spec isn't one of [GoBILDA]'s presets -- clones the SDK's motor config with a custom [cpr]/[rpm]. */
    constructor(hardwareMap: HardwareMap, id: String, cpr: Double, rpm: Double) : this(hardwareMap, id, GoBILDA.NONE) {
        val configType = motor.motorType.clone()
        configType.setMaxRPM(rpm)
        configType.setTicksPerRev(cpr)
        motor.motorType = configType
        achievableMaxTicksPerSecond = cpr * rpm / 60
    }

    enum class GoBILDA(val cpr: Double, val rpm: Double) {
        RPM_30(5264.0, 30.0), RPM_43(3892.0, 43.0), RPM_60(2786.0, 60.0), RPM_84(1993.6, 84.0),
        RPM_117(1425.2, 117.0), RPM_223(753.2, 223.0), RPM_312(537.6, 312.0), RPM_435(383.6, 435.0),
        RPM_1150(145.6, 1150.0), RPM_1620(103.6, 1620.0), BARE(28.0, 6000.0), NONE(0.0, 0.0);

        val achievableMaxTicksPerSecond get() = cpr * rpm / 60
    }

    enum class RunMode { VELOCITY_CONTROL, POSITION_CONTROL, RAW_POWER }

    enum class ZeroPowerBehavior(val sdkBehavior: DcMotor.ZeroPowerBehavior) {
        UNKNOWN(DcMotor.ZeroPowerBehavior.UNKNOWN),
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT),
    }

    /** Wraps the motor's raw encoder tick count with velocity/acceleration estimation and a configurable distance-per-pulse. */
    open inner class Encoder(private val position: () -> Int) {

        internal var resetVal = 0
        private var lastPosition = 0
        private var direction = 1
        internal var dpp = 1.0
        private var lastTimestamp = System.nanoTime() / 1e9
        private var velocityEstimate = 0.0
        private var lastVelocity = 0.0
        private var acceleration = 0.0

        open fun getPosition(): Int {
            val current = position()
            if (current != lastPosition) {
                val now = System.nanoTime() / 1e9
                val dt = now - lastTimestamp
                velocityEstimate = (current - lastPosition) / dt
                lastPosition = current
                lastTimestamp = now
            }
            return direction * current - resetVal
        }

        val distance get() = dpp * getPosition()
        val rate get() = dpp * getRawVelocity()

        fun reset() {
            resetVal += getPosition()
        }

        fun overrideResetPosition(position: Int) = apply { resetVal = position }
        fun setDistancePerPulse(distancePerPulse: Double) = apply { dpp = distancePerPulse }
        fun setDirection(reversed: Boolean) = apply { direction = if (reversed) -1 else 1 }

        val revolutions get() = getPosition() / cpr

        fun getRawVelocity(): Double {
            // Calls the enclosing Motor's (possibly overridden, e.g. by MotorEx) getVelocity().
            val velocity = getVelocity()
            val now = System.nanoTime() / 1e9
            val dt = now - lastTimestamp
            if (dt > 1e-4) {
                acceleration = (velocity - lastVelocity) / dt
                lastVelocity = velocity
                lastTimestamp = now
            }
            return velocity * direction
        }

        fun getAcceleration() = acceleration

        /** [getRawVelocity], corrected for the SDK's 16-bit velocity-overflow wraparound. */
        fun getCorrectedVelocity(): Double {
            val real = getRawVelocity()
            var abs = kotlin.math.abs(real)
            while (kotlin.math.abs(velocityEstimate - abs) > CPS_STEP / 2.0) {
                abs += kotlin.math.sign(velocityEstimate - abs) * CPS_STEP
            }
            return abs * kotlin.math.sign(real)
        }
    }

    val encoder: Encoder = Encoder(motor::getCurrentPosition)

    var runMode: RunMode = RunMode.RAW_POWER
        protected set

    var achievableMaxTicksPerSecond: Double = if (type == GoBILDA.NONE) (motor as? DcMotorEx)?.motorType?.achieveableMaxTicksPerSecond ?: 0.0 else type.achievableMaxTicksPerSecond
        protected set

    protected val veloController = PIDFController(1.0, 0.0, 0.0, 0.0)
    protected val positionController = PIDFController(1.0, 0.0, 0.0, 0.0)
    protected var feedforward = SimpleMotorFeedforward(0.0, 1.0, 0.0)

    private var targetIsSet = false
    protected var bufferFraction = 0.9
    protected var lastPower = 0.0

    /** Sets the motor's output power/velocity/position target, depending on [runMode]. [output] is in `[-1, 1]` for [RunMode.RAW_POWER]. */
    open fun set(output: Double) {
        val power = when (runMode) {
            RunMode.VELOCITY_CONTROL -> {
                val speed = bufferFraction * output * achievableMaxTicksPerSecond
                (veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration())) / achievableMaxTicksPerSecond
            }
            RunMode.POSITION_CONTROL -> output * positionController.calculate(encoder.distance)
            RunMode.RAW_POWER -> output
        }
        motor.power = power
        lastPower = power
    }

    open fun setDistancePerPulse(distancePerPulse: Double): Encoder = encoder.setDistancePerPulse(distancePerPulse)

    val distance get() = encoder.distance
    val rate get() = encoder.rate

    open fun atTargetPosition() = positionController.atSetPoint()

    open fun resetEncoder() = encoder.reset()

    open fun stopAndResetEncoder() {
        encoder.resetVal = 0
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    val veloCoefficients get() = veloController.coefficients
    val positionCoefficient get() = positionController.p
    val feedforwardCoefficients get() = doubleArrayOf(feedforward.ks, feedforward.kv, feedforward.ka)

    open fun setZeroPowerBehavior(behavior: ZeroPowerBehavior) = apply { motor.zeroPowerBehavior = behavior.sdkBehavior }

    open val currentPosition get() = encoder.getPosition()
    open fun getCorrectedVelocity() = encoder.getCorrectedVelocity()

    open val cpr get() = if (type == GoBILDA.NONE) (motor as? DcMotorEx)?.motorType?.ticksPerRev ?: 0.0 else type.cpr
    open val maxRpm get() = if (type == GoBILDA.NONE) (motor as? DcMotorEx)?.motorType?.maxRPM ?: 0.0 else type.rpm

    /** How much of a step towards the velocity setpoint is taken per [set] call, in `(0, 1]`. */
    fun setBuffer(fraction: Double) {
        require(fraction in 0.0..1.0 && fraction != 0.0) { "Buffer must be between 0 and 1, exclusive to 0" }
        bufferFraction = fraction
    }

    open fun setRunMode(mode: RunMode) = apply {
        runMode = mode
        veloController.reset()
        positionController.reset()
        if (mode == RunMode.POSITION_CONTROL && !targetIsSet) {
            setTargetPosition(currentPosition)
            targetIsSet = false
        }
    }

    protected open fun getVelocity(): Double = (motor as DcMotorEx).velocity

    /** The last power [set] wrote to this object -- not necessarily what's currently on the motor (see [getRawPower]). */
    open fun get() = lastPower

    open fun getRawPower(): Double = motor.power

    open fun setTargetPosition(target: Int) = setTargetDistance(target * encoder.dpp)

    open fun setTargetDistance(target: Double) {
        targetIsSet = true
        positionController.setPoint = target
    }

    open fun setPositionTolerance(tolerance: Double) = positionController.setTolerance(tolerance)

    open fun setInverted(isInverted: Boolean) = apply {
        motor.direction = if (isInverted) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }

    open fun getInverted() = motor.direction == DcMotorSimple.Direction.REVERSE

    open fun setVeloCoefficients(kp: Double, ki: Double, kd: Double) = veloController.setPIDF(kp, ki, kd, 0.0)

    open fun setFeedforwardCoefficients(ks: Double, kv: Double, ka: Double = 0.0) {
        feedforward = SimpleMotorFeedforward(ks, kv, ka)
    }

    open fun setPositionCoefficient(kp: Double) {
        positionController.p = kp
    }

    override fun disable() = motor.close()

    override fun getDeviceType() = "Motor ${motor.deviceName} from ${motor.manufacturer} in port ${motor.portNumber}"

    open fun stopMotor() {
        motor.power = 0.0
    }

    companion object {
        private const val CPS_STEP = 0x10000
    }
}
