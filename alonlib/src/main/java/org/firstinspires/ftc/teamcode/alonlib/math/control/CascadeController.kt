package org.firstinspires.ftc.teamcode.alonlib.math.control

import kotlin.math.abs

/**
 * A cascaded position -> velocity controller: [primary] converts position error into a velocity
 * setpoint, which [secondary] then chases. Useful when you want a smoother, velocity-limited
 * approach to a position setpoint than a single position PID gives you.
 *
 * Note [primary]/[secondary] are driven directly by this controller rather than through their own
 * [calculate] loops -- don't also call them independently.
 */
class CascadeController(private val primary: PIDFController, private val secondary: PIDFController) : PIDFController(0.0, 0.0, 0.0, 0.0) {

    var measuredVelocity = 0.0
        private set

    private var prevMeasuredValue = 0.0
    private var velocitySetPoint = 0.0

    /** Sets both the position setpoint [positionSetPoint] and the velocity setpoint [velocitySetPoint]. */
    fun setSetPoints(positionSetPoint: Double, velocitySetPoint: Double = 0.0) {
        setPoint = positionSetPoint
        this.velocitySetPoint = velocitySetPoint
        positionError = setPoint - measuredValue
        velocityError = this.velocitySetPoint - measuredVelocity
    }

    override fun calculateOutput(pv: Double): Double {
        prevError = positionError

        val now = System.nanoTime() / 1e9
        if (lastTimeStamp == 0.0) {
            lastTimeStamp = now
            prevMeasuredValue = pv
        }
        period = now - lastTimeStamp

        measuredValue = pv
        positionError = setPoint - measuredValue

        if (abs(period) > 1e-6) {
            measuredVelocity = (measuredValue - prevMeasuredValue) / period
            prevMeasuredValue = measuredValue
            lastTimeStamp = now
        }

        velocityError = velocitySetPoint - measuredVelocity

        val velocityGoal = primary.calculate(pv, setPoint)
        return secondary.calculate(measuredVelocity, velocityGoal + velocitySetPoint)
    }

    override fun reset() {
        super.reset()
        measuredVelocity = 0.0
        primary.reset()
        secondary.reset()
    }
}
