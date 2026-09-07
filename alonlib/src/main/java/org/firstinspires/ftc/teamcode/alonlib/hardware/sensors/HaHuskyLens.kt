package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.hardware.HardwareMap

/** The DFRobot HuskyLens AI vision sensor: onboard object/tag/color/line recognition, in whichever [HuskyLens.Algorithm] mode is selected. */
class HaHuskyLens(val huskyLens: HuskyLens) : com.qualcomm.robotcore.hardware.HardwareDevice by huskyLens {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(HuskyLens::class.java, id))

    fun knock() = huskyLens.knock()

    fun selectAlgorithm(algorithm: HuskyLens.Algorithm) = huskyLens.selectAlgorithm(algorithm)

    fun blocks(): Array<HuskyLens.Block> = huskyLens.blocks()
    fun blocks(id: Int): Array<HuskyLens.Block> = huskyLens.blocks(id)

    fun arrows(): Array<HuskyLens.Arrow> = huskyLens.arrows()
    fun arrows(id: Int): Array<HuskyLens.Arrow> = huskyLens.arrows(id)
}
