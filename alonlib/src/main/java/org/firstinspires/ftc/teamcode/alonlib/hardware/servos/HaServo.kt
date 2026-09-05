package org.firstinspires.ftc.teamcode.alonlib.hardware.servos

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Mode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Type
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.mapRange
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

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
                value.coerceIn(minPercentOutput..maxPercentOutput)
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
     * the maximum [position] to be sent to the servo
     */
    var maxPosition: Rotation2d = halfRange
        set(value) {
            field = value.degrees.coerceIn(-halfRange.normalizedDegrees..halfRange.normalizedDegrees).degrees
            followers.forEach { it.maxPosition = field }
        }

    /**
     * the minimum [position] to be sent to the servo
     */
    var minPosition: Rotation2d = (-halfRange)
        set(value) {
            field = value.degrees.coerceIn(-halfRange.normalizedDegrees..maxPosition.normalizedDegrees).degrees
            followers.forEach { it.minPosition = field }
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
                Mode.Cr        -> robotPrintError("cannot set position in CR mode")
                Mode.FullRange -> {
                    field = position
                    servo.position = mapRange(
	                    position.normalizedDegrees.coerceIn(minPosition.normalizedDegrees, maxPosition.normalizedDegrees),
	                    0.0,
	                    type.range.normalizedDegrees,
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
                Mode.Cr -> {
                    servo.position = mapRange(value.asRpm, minVelocity.asRpm, maxVelocity.asRpm, 0.0, 1.0)
                    field = value
                    followers.forEach { it.velocity = value }
                }

                Mode.FullRange -> robotPrintError("cannot set velocity in full range mode")

            }
        }


    fun stop() {
        when (mode) {
            Mode.Cr        -> percentOutput = 0.0
            Mode.FullRange -> {}
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
