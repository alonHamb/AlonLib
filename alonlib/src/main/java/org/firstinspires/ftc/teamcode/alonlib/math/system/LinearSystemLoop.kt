package org.firstinspires.ftc.teamcode.alonlib.math.system

import org.firstinspires.ftc.teamcode.alonlib.math.control.LinearPlantInversionFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.control.LinearQuadraticRegulator
import org.firstinspires.ftc.teamcode.alonlib.math.estimator.KalmanFilter

/**
 * Combines a [controller], [feedforward], and [observer] into one full-state-feedback control
 * loop for a [plant]. "Inputs"/"outputs" are from the plant's perspective throughout (`u` is what
 * you send to the motors, `y` is what comes back from sensors).
 */
class LinearSystemLoop(
    private val controller: LinearQuadraticRegulator,
    private val feedforward: LinearPlantInversionFeedforward,
    val observer: KalmanFilter,
    private var clampFunction: (Matrix) -> Matrix,
) {

    constructor(plant: LinearSystem, controller: LinearQuadraticRegulator, observer: KalmanFilter, maxVoltageVolts: Double, dtSeconds: Double) :
            this(controller, LinearPlantInversionFeedforward(plant, dtSeconds), observer, maxVoltageVolts)

    constructor(
        controller: LinearQuadraticRegulator,
        feedforward: LinearPlantInversionFeedforward,
        observer: KalmanFilter,
        maxVoltageVolts: Double,
    ) : this(controller, feedforward, observer, { u -> StateSpaceUtil.desaturateInputVector(u, maxVoltageVolts) })

    private var nextR: Matrix = Matrix(controller.k.cols, 1)

    init {
        reset(nextR)
    }

    /** The controller's calculated (clamped) control input `u`, plus the feedforward. */
    fun u(): Matrix = clampInput(controller.u + feedforward.uff)

    /** Zeroes the reference and controller output, and resets the feedforward/observer to [initialState]. */
    fun reset(initialState: Matrix) {
        nextR.fill(0.0)
        controller.reset()
        feedforward.reset(initialState)
        observer.xHat = initialState
    }

    /** The difference between the reference and the observer's current state estimate. */
    fun error(): Matrix = controller.r - observer.xHat

    /** Corrects the observer's state estimate using measurement [y]. */
    fun correct(y: Matrix) = observer.correct(u(), y)

    /** Sets a new controller output, projects the model forward, and runs observer prediction, over [dtSeconds]. */
    fun predict(dtSeconds: Double) {
        val u = clampInput(controller.calculate(observer.xHat, nextR) + feedforward.calculate(nextR))
        observer.predict(u, dtSeconds)
    }

    fun setNextR(nextR: Matrix) {
        this.nextR = nextR
    }

    private fun clampInput(unclampedU: Matrix) = clampFunction(unclampedU)
}
