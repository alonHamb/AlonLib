package org.firstinspires.ftc.teamcode.alonlib.math.control

/**
 * A [PIDController] whose setpoint is constrained by a [TrapezoidProfile] instead of being
 * tracked directly -- call [reset] before first use to avoid a spurious jump from `(0, 0)`.
 */
class ProfiledPIDController(kp: Double, ki: Double, kd: Double, var constraints: TrapezoidProfile.Constraints) {

    private val controller = PIDController(kp, ki, kd)

    var goal = TrapezoidProfile.State()
        private set

    var setpoint = TrapezoidProfile.State()
        private set

    fun setPID(kp: Double, ki: Double, kd: Double) = controller.setPID(kp, ki, kd)

    var p
        get() = controller.p
        set(value) {
            controller.p = value
        }

    var i
        get() = controller.i
        set(value) {
            controller.i = value
        }

    var d
        get() = controller.d
        set(value) {
            controller.d = value
        }

    val period get() = controller.period

    fun setGoal(goal: TrapezoidProfile.State) {
        this.goal = goal
    }

    fun setGoal(goal: Double) {
        this.goal = TrapezoidProfile.State(goal, 0.0)
    }

    val atGoal get() = atSetpoint && goal == setpoint

    val atSetpoint get() = controller.atSetPoint()

    fun setTolerance(positionTolerance: Double, velocityTolerance: Double = Double.POSITIVE_INFINITY) =
        controller.setTolerance(positionTolerance, velocityTolerance)

    val positionError get() = controller.positionError
    val velocityError get() = controller.velocityError

    /** Advances the trapezoid-profiled setpoint towards [goal] and returns the next PID output for [measurement]. */
    fun calculate(measurement: Double): Double {
        val profile = TrapezoidProfile(constraints)
        setpoint = profile.calculate(period, setpoint, goal)
        return controller.calculate(measurement, setpoint.position)
    }

    fun calculate(measurement: Double, goal: TrapezoidProfile.State): Double {
        setGoal(goal)
        return calculate(measurement)
    }

    fun calculate(measurement: Double, goal: Double): Double {
        setGoal(goal)
        return calculate(measurement)
    }

    fun calculate(measurement: Double, goal: TrapezoidProfile.State, constraints: TrapezoidProfile.Constraints): Double {
        this.constraints = constraints
        return calculate(measurement, goal)
    }

    /** Resets the previous error and integral term, and disables (i.e. doesn't reset) the setpoint/goal. */
    fun reset() = controller.reset()

    fun reset(measurement: TrapezoidProfile.State) {
        controller.reset()
        setpoint = measurement
    }

    fun reset(measuredPosition: Double, measuredVelocity: Double = 0.0) = reset(TrapezoidProfile.State(measuredPosition, measuredVelocity))
}
