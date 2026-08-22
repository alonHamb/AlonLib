package org.firstinspires.ftc.teamcode.alonlib.hardware.servos

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.Direction
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Mode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Type
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import kotlin.math.abs

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
    /**
     * the base [servo] object
     *
     * NOT TO BE USED UNLESS YOU KNOW WHAT YOU ARE DOING
     */
    var servo: Servo = hardwareMap.get(Servo::class.java, id)

    init {
        (servo as ServoImplEx).apply {
            pwmRange = (PwmControl.PwmRange(500.0, 2500.0))
        }
    }

    fun setPwm(pwmRange: PwmControl.PwmRange) = apply { controller.setServoPwmRange(servo.portNumber, pwmRange) }

    val controller: ServoControllerEx get() = servo.controller as ServoControllerEx

    /** the minimum position delta (or exactly zero) before a write actually reaches the servo */
    var cachingTolerance = 0.0001
    private var lastWrittenPosition = Double.NaN

    private fun writePosition(pos: Double) {
        if (lastWrittenPosition.isNaN() || abs(pos - lastWrittenPosition) > cachingTolerance) {
            servo.position = pos
            lastWrittenPosition = pos
            followers.forEach { it.writePosition(pos) }
        }
    }


    // --- state getters and setters ---

    /**
     * software forward limit only for [percentOutput]
     */
    var forwardLimit = { false }

    /**
     * software reverse limit only for [percentOutput]
     */
    var reverseLimit = { false }

    /**
     * the maximum output to be sent to the servo when set with [percentOutput]
     */
    var maxPercentOutput = 1.0
        set(value) {
            field = value.coerceIn(0.0..1.0)
        }

    /**
     * the minimum output to be sent to the servo when set with [percentOutput]
     */
    var minPercentOutput = 0.0
        set(value) {
            field = value.coerceIn(0.0..maxPercentOutput)
        }

    /**
     * a way to control the output to the servo as a percent
     */
    var percentOutput: Double = 0.0
        set(value) {
            if (!(forwardLimit() && value > 0) && !(reverseLimit() && value < 0)) {
                field = value
                writePosition(value.coerceIn(minPercentOutput..maxPercentOutput))
            } else {
                robotPrintError("limit reached")
            }
        }
        get() = servo.position

    /**
     * half of [Type.range], in degrees -- the most a [Rotation2d] can represent [position] as an
     * offset from the center of the servo's sweep without exceeding the (-180, 180] domain that
     * Rotation2d normalizes into (350deg/2 = 175deg, the widest built-in [Type]). [position],
     * [minPosition], and [maxPosition] are all relative to this center, i.e. 0 degrees is the
     * middle of the servo's travel, not one end of it.
     */
    private val halfRange = type.range / 2.0

    /**
     * the maximum [position] to be sent to the servo, relative to the center of its sweep
     */
    var maxPosition: Rotation2d = halfRange.degrees
        set(value) {
            field = value.degrees.coerceIn(-halfRange..halfRange).degrees
        }

    /**
     * the minimum [position] to be sent to the servo, relative to the center of its sweep
     */
    var minPosition: Rotation2d = (-halfRange).degrees
        set(value) {
            field = value.degrees.coerceIn(-halfRange..maxPosition.degrees).degrees
        }


    /**
     * a soft limit, in degrees measured from the low end of the physical sweep -- e.g. straight off
     * the servo's datasheet -- rather than [position]'s center-relative [Rotation2d]. Restricts how
     * far [position] can move in the negative direction, on top of (not instead of) [minPosition].
     * Defaults to 0deg, the physical low end, i.e. no extra restriction.
     */
    var minLimit: Double = 0.0
        set(value) {
            field = value.coerceIn(0.0..maxLimit)
        }

    /**
     * a soft limit, in degrees measured from the low end of the physical sweep -- e.g. straight off
     * the servo's datasheet -- rather than [position]'s center-relative [Rotation2d]. Restricts how
     * far [position] can move in the positive direction, on top of (not instead of) [maxPosition].
     * Defaults to [Type.range], the physical high end, i.e. no extra restriction.
     */
    var maxLimit: Double = type.range
        set(value) {
            field = value.coerceIn(minLimit..type.range)
        }

    /**
     * when called returns the last [position] that have been sent to the servo
     *
     * when set sets the [position] you want the servo to go to, relative to the center of its
     * sweep (0 degrees = centered, not one end of travel -- see [halfRange])
     */
    var position: Rotation2d = 0.0.degrees
        set(position) {
            when (mode) {
                Mode.CR         -> robotPrintError("cannot set position in CR mode")
                Mode.FULL_RANGE -> {
                    val hardwareClampedDegrees = position.degrees.coerceIn(minPosition.degrees, maxPosition.degrees)
                    val absoluteDegrees = (hardwareClampedDegrees + halfRange).coerceIn(minLimit, maxLimit)
                    writePosition(absoluteDegrees / type.range)
                    field = (absoluteDegrees - halfRange).degrees
                }
            }
        }

    /**
     * the maximum [velocity] to be sent to the servo
     */
    var maxVelocity: AngularVelocity = type.maxSpeed
        set(value) {
            field = value.asRpm.coerceIn(0.0..type.maxSpeed.asRpm).rpm
        }

    /**
     * the minimum [velocity] to be sent to the servo
     */
    var minVelocity: AngularVelocity = 0.0.rpm
        set(value) {
            field = value.asRpm.coerceIn(0.0..maxVelocity.asRpm).rpm
        }

    /**
     * when called returns the last [velocity] that have been sent to the servo
     *
     * when set sets the [velocity] you want the servo to get to
     */
    var velocity: AngularVelocity = 0.0.rpm
        set(value) {
            when (mode) {
                Mode.CR         -> {
                    writePosition(value.asRpm.coerceIn(minVelocity.asRpm..maxVelocity.asRpm) / type.maxSpeed.asRpm)
                    field = value
                }

                Mode.FULL_RANGE -> robotPrintError("cannot set velocity in full range mode")

            }
        }

    /**
     * the direction of the servo
     */
    var runningDirection: Direction
        get() {
            return when (servo.direction) {
                Servo.Direction.FORWARD -> Direction.REVERSE
                Servo.Direction.REVERSE -> Direction.FORWARD
            }
        }
        set(runningDirection) {
            when (runningDirection) {
                Direction.FORWARD -> servo.direction = Servo.Direction.FORWARD
                Direction.REVERSE -> servo.direction = Servo.Direction.REVERSE
            }
        }

    fun stop() {
        when (mode) {
            Mode.CR         -> percentOutput = 0.0
            Mode.FULL_RANGE -> {}
        }
        followers.forEach { it.stop() }
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String {
        return "HaServo"
    }

    override fun getConnectionInfo(): String {
        return ""
    }

    override fun getVersion(): Int {
        return 1
    }

    override fun resetDeviceConfigurationForOpMode() {
    }

    override fun close() {
        servo.close()
        followers.forEach { it.close() }
    }


}
