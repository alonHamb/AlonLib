package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Mode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Type
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

class HaServo(
    hardwareMap: HardwareMap,
    id: String,
    val mode: Mode,
    val type: Type
) : HardwareDevice {
    // --- servo object declaration ---
    /**
     * the base [servo] object
     *
     * NOT TO BE USED UNLESS YOU KNOW WHAT YOU ARE DOING
     */
    val servo: Servo = hardwareMap.get(Servo::class.java, id)

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
            if (!(forwardLimit() && percentOutput > 0) or !(reverseLimit() && percentOutput < 0)) {
                field = percentOutput
                servo.position = value.coerceIn(minPercentOutput..maxPercentOutput)
            } else {
                robotPrintError("limit reached")
            }
        }
        get() = servo.position

    /**
     * the maximum [position] to be sent to the servo
     */
    var maxPosition: Rotation2d = type.range
        set(value) {
            field = value.degrees.coerceIn(0.0..type.range.degrees).degrees
        }

    /**
     * the minimum [position] to be sent to the motor
     */
    var minPosition: Rotation2d = 0.0.degrees
        set(value) {
            field = value.degrees.coerceIn(0.0..maxPosition.degrees).degrees
        }


    /**
     * when called returns the last [position] that have been sent to the servo
     *
     * when set sets the [position] you want the servo to go to
     */
    var position: Rotation2d = 0.0.degrees
        set(position) {
            when (mode) {
                Mode.CR -> robotPrintError("cannot set position in CR mode")
                Mode.FULL_RANGE -> {
                    servo.position = MathUtils.normalizeDegrees(
                        (position.degrees.coerceIn(
                            MathUtils.normalizeDegrees(minPosition.degrees, true),
                            MathUtils.normalizeDegrees(maxPosition.degrees, true)
                        ) / (MathUtils.normalizeDegrees(type.range.degrees, true))),
                        true
                    )
                    field = type.range * servo.position
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
                Mode.CR -> {
                    servo.position = (value.asRpm.coerceIn(minVelocity.asRpm..maxVelocity.asRpm) / type.maxSpeed.asRpm)
                    field = value
                }

                Mode.FULL_RANGE -> robotPrintError("cannot set velocity in full range mode")

            }
        }

    /**
     * the direction of the servo
     */
    var runningDirection: Motor.Direction
        get() {
            return when (servo.direction) {
                Servo.Direction.FORWARD -> Motor.Direction.REVERSE
                Servo.Direction.REVERSE -> Motor.Direction.FORWARD
            }
        }
        set(runningDirection) {
            when (runningDirection) {
                Motor.Direction.FORWARD -> servo.direction = Servo.Direction.FORWARD
                Motor.Direction.REVERSE -> servo.direction = Servo.Direction.REVERSE
            }
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
    }


}
