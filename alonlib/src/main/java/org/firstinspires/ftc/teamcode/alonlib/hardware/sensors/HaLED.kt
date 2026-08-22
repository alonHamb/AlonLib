package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.Blinker
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Light
import com.qualcomm.robotcore.hardware.SwitchableLight

/**
 * Covers any of the SDK's [Light]/[SwitchableLight]/[Blinker] interfaces -- unlike most `Ha*`
 * wrappers, none of these extend the SDK's `HardwareDevice`, so this doesn't either. Methods
 * specific to one of the three throw if the underlying device doesn't support it.
 */
class HaLED(private val device: Any) {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(SwitchableLight::class.java, id))

    val isOn get() = (device as Light).isLightOn

    /** Requires the device to be a [SwitchableLight]. */
    fun setOn(on: Boolean) = (device as SwitchableLight).enableLight(on)

    /** Requires the device to be a [Blinker]. */
    fun setPattern(pattern: Collection<Blinker.Step>) {
        (device as Blinker).pattern = pattern
    }

    /** Requires the device to be a [Blinker]. */
    fun setConstantColor(color: Int) = (device as Blinker).setConstant(color)

    /** Requires the device to be a [Blinker]. */
    fun stopBlinking() = (device as Blinker).stopBlinking()
}
