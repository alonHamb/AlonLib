package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

/** Links a [leader] CR servo and its [followers] into one [CRServo]: the leader is driven normally, and every follower mirrors its output. */
class CRServoGroup(private val leader: CRServo, private vararg val followers: CRServo) : CRServo(leader.crServo), Iterable<CRServo> {

    private val group: List<CRServo> = listOf(leader, *followers)

    override fun set(output: Double) {
        leader.set(output)
        followers.forEach { it.set(leader.get()) }
    }

    override fun get() = leader.get()
    override fun getRawPower() = leader.getRawPower()

    val speeds get() = group.map { it.get() }
    val rawPowers get() = group.map { it.getRawPower() }

    override fun iterator() = group.iterator()

    override fun getInverted() = leader.getInverted()

    override fun setInverted(isInverted: Boolean) = apply { group.forEach { it.setInverted(isInverted) } }

    override fun disable() = group.forEach { it.disable() }
    override fun getDeviceType() = "CRServo Group"
    override fun stopMotor() = group.forEach { it.stopMotor() }
}
