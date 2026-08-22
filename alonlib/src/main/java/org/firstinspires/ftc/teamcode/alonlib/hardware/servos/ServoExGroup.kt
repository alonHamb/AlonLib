package org.firstinspires.ftc.teamcode.alonlib.hardware.servos

/** Links a [leader] servo and its [followers] into one [ServoEx]: the leader is driven normally, and every follower mirrors its output. */
class ServoExGroup(private val leader: ServoEx, private vararg val followers: ServoEx) : ServoEx(leader.getServo()), Iterable<ServoEx> {

    private val group: List<ServoEx> = listOf(leader, *followers)

    override fun set(output: Double) {
        leader.set(output)
        followers.forEach { it.set(leader.get()) }
    }

    override fun get() = leader.get()

    val positions get() = group.map { it.get() }

    override fun iterator() = group.iterator()

    override fun getInverted() = leader.getInverted()

    override fun setInverted(inverted: Boolean) = apply { group.forEach { it.setInverted(inverted) } }

    override fun disable() = group.forEach { it.disable() }
    override fun getDeviceType() = "ServoEx Group"
}
