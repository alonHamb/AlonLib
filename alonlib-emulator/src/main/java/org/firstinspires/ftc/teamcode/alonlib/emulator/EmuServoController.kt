package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType
import emulator.hardware.SimServo

/**
 * Backs one hub's worth of [SimServo]s. `hardwareMap.get(Servo::class.java, id)` must return a
 * real [ServoImplEx] -- [org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo]
 * unconditionally force-casts to it -- so rather than faking `Servo`/`ServoImplEx` themselves
 * (the latter is a concrete class), this only fakes the [ServoControllerEx] a genuine
 * `ServoImplEx` delegates every operation to; see [emulatedServo].
 */
class EmuServoController(private val portsToSims: Map<Int, SimServo>) : ServoControllerEx {
    private val pwmRanges = HashMap<Int, PwmControl.PwmRange>()
    private val pwmEnabled = HashMap<Int, Boolean>()

    private fun sim(port: Int) = portsToSims.getValue(port)

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuServoController"
    override fun getConnectionInfo(): String = "emulated"
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}

    override fun pwmEnable() {}
    override fun pwmDisable() {}
    override fun getPwmStatus(): ServoController.PwmStatus = ServoController.PwmStatus.ENABLED

    override fun setServoPosition(servo: Int, position: Double) = sim(servo).setPosition(position)
    override fun getServoPosition(servo: Int): Double = sim(servo).getPosition()

    override fun setServoPwmRange(servo: Int, range: PwmControl.PwmRange) {
        pwmRanges[servo] = range
    }

    override fun getServoPwmRange(servo: Int): PwmControl.PwmRange = pwmRanges[servo] ?: PwmControl.PwmRange.defaultRange
    override fun setServoPwmEnable(servo: Int) {
        pwmEnabled[servo] = true
    }

    override fun setServoPwmDisable(servo: Int) {
        pwmEnabled[servo] = false
    }

    override fun isServoPwmEnabled(servo: Int): Boolean = pwmEnabled[servo] ?: true
    override fun setServoType(servo: Int, servoType: ServoConfigurationType) {}
}

/** Builds a genuine [ServoImplEx] for hub port [port], backed by that port's [SimServo]. */
fun emulatedServo(controller: EmuServoController, port: Int): ServoImplEx =
    EmulatedServoImplEx(controller, port, ServoConfigurationType())

/**
 * [ServoImplEx] as constructed is otherwise fully real -- position/direction/PWM all flow through
 * [EmuServoController] into simulated dynamics -- except [ServoImpl.getDeviceName], which calls
 * into `AppUtil.getDefContext()`, a real Android resource lookup that isn't available on a
 * desktop JVM. Overridden here since nothing in AlonLib needs the real value.
 */
private class EmulatedServoImplEx(
    controller: EmuServoController,
    port: Int,
    type: ServoConfigurationType
) : ServoImplEx(controller, port, type) {
    override fun getDeviceName(): String = "EmuServo"
}
