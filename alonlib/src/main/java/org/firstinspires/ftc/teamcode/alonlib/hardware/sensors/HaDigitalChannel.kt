package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

/** A single digital I/O pin (e.g. a beam-break or a limit switch wired directly, not through a [com.qualcomm.robotcore.hardware.TouchSensor]). */
class HaDigitalChannel(val digitalChannel: DigitalChannel) : com.qualcomm.robotcore.hardware.HardwareDevice by digitalChannel {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(DigitalChannel::class.java, id))

    var mode: DigitalChannel.Mode
        get() = digitalChannel.mode
        set(value) { digitalChannel.mode = value }

    var state: Boolean
        get() = digitalChannel.state
        set(value) { digitalChannel.setState(value) }
}
