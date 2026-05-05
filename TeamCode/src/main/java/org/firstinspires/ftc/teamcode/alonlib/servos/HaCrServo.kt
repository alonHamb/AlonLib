package org.firstinspires.ftc.teamcode.alonlib.motors

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx.RunMode
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput

class HaCrServo(hardwareMap: HardwareMap, id: String) : HardwareDevice {


    // --- hardware declaration ---
    val crServo = CRServoEx(hardwareMap, id).apply { runMode = RunMode.RawPower }

    // --- motor configurations ---

    /**
    sets the behavior of the motor when stop() is called or when you set [percentOutput] to zero
     */
    var zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        set(value) {
            crServo.setZeroPowerBehavior(value)
            field = value
        }

    /**
     * the direction the motor to rotates
     * @param Motor.Direction.FORWARD clockwise
     * @param Motor.Direction.REVERSE counterclockwise
     */
    var runningDirection: Motor.Direction
        get() {
            return when (crServo.inverted) {
                true -> Motor.Direction.REVERSE
                false -> Motor.Direction.FORWARD
            }
        }
        set(value) {
            when (value) {
                Motor.Direction.FORWARD -> crServo.inverted = false
                Motor.Direction.REVERSE -> crServo.inverted = true
            }
        }
    
    var runMode: RunMode = RunMode.RawPower
        set(value) {
            crServo.setRunMode(runMode)
            field = value
        }


    // --- state getters and setters ---

    /**
     * sets the percent output of the motor.
     * is clamped between properties [minPercentOutput] and [maxPercentOutput].
     * default is -1.0 and 1.0
     */
    var percentOutput: PercentOutput = 0.0
        get() = crServo.get()
        set(percentOutput) {
            if (!(forwardLimit() && percentOutput > 0) or !(reverseLimit() && percentOutput < 0)) {
                field = percentOutput
                crServo.setRunMode(RunMode.RawPower)
                crServo.set(percentOutput.coerceIn(minPercentOutput, maxPercentOutput))
                crServo.setRunMode(runMode)
            } else {
                robotPrintError("limit reached")
            }

        }


    /**
     * Software forward limit, ONLY for percent-output control.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software reverse limit, ONLY for percent-output control.
     */
    var reverseLimit: () -> Boolean = { false }


    // --- limits ---

    /**
     * the smallest number you can sed to the motor with the [percentOutput] property
     */
    var minPercentOutput = -1.0
        set(percentOutput) {
            field = percentOutput.coerceIn(-1.0, maxPercentOutput)
        }

    /**
     * the largest number you can send to the motor with the [percentOutput] property
     */
    var maxPercentOutput = 1.0
        set(percentOutput) {
            field = percentOutput.coerceIn(minPercentOutput, 1.0)
        }


    /**
     * stops the motor
     *
     * does the same as setting [percentOutput] to 0.0
     */
    fun stop() {
        percentOutput = 0.0
        crServo.stopMotor()
    }

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
        crServo.stopAndResetEncoder()
    }

    override fun close() {
        crServo.disable()
    }


}
