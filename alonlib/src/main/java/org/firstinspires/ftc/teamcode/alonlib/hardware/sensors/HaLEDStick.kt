package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.hardware.sparkfun.SparkFunLEDStick
import com.qualcomm.robotcore.hardware.HardwareMap

/** The SparkFun addressable LED stick: per-pixel color/brightness control. */
class HaLEDStick(val ledStick: SparkFunLEDStick) : com.qualcomm.robotcore.hardware.HardwareDevice by ledStick {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(SparkFunLEDStick::class.java, id))

    /** Sets every pixel to [color]. */
    fun setColor(color: Int) = ledStick.setColor(color)

    /** Sets pixel [position] to [color]. */
    fun setColor(position: Int, color: Int) = ledStick.setColor(position, color)

    /** Sets every pixel individually, one color per element. */
    fun setColors(colors: IntArray) = ledStick.setColors(colors)

    fun setBrightness(brightness: Int) = ledStick.setBrightness(brightness)
    fun setBrightness(position: Int, brightness: Int) = ledStick.setBrightness(position, brightness)

    fun turnAllOff() = ledStick.turnAllOff()
}
