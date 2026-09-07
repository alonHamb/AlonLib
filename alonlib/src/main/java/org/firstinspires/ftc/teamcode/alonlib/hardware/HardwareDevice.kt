package org.firstinspires.ftc.teamcode.alonlib.hardware

/** Common surface for AlonLib's higher-level hardware wrappers (motors, servos, sensors). */
interface HardwareDevice {
    /** Releases the underlying FTC SDK device. */
    fun disable()

    /** A human-readable description of this device, for logging. */
    fun getDeviceType(): String
}
