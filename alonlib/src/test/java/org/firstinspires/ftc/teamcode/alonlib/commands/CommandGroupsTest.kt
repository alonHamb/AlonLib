package org.firstinspires.ftc.teamcode.alonlib.commands

import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class CommandGroupsTest {

    @Before
    fun setUp() = CommandScheduler.reset()

    @After
    fun tearDown() = CommandScheduler.reset()

    private fun finishAfter(executions: Int, onExecute: () -> Unit = {}): CommandBase {
        var count = 0
        return FunctionalCommand({}, { count++; onExecute() }, {}, { count >= executions })
    }

    @Test
    fun `SequentialCommandGroup runs its commands one after another`() {
        val order = mutableListOf<Int>()
        val group = SequentialCommandGroup(
            finishAfter(1) { order.add(1) },
            finishAfter(1) { order.add(2) },
        )

        group.schedule()
        CommandScheduler.run() // first command executes and finishes
        assertEquals(listOf(1), order)
        assertTrue(group.isScheduled())

        CommandScheduler.run() // second command executes and finishes
        assertEquals(listOf(1, 2), order)
        assertFalse(group.isScheduled())
    }

    @Test
    fun `ParallelCommandGroup finishes only once every command has`() {
        val group = ParallelCommandGroup(finishAfter(1), finishAfter(2))

        group.schedule()
        CommandScheduler.run()
        assertTrue(group.isScheduled())

        CommandScheduler.run()
        assertFalse(group.isScheduled())
    }

    @Test
    fun `ParallelRaceGroup ends as soon as the first command finishes`() {
        val group = ParallelRaceGroup(finishAfter(1), finishAfter(100))

        group.schedule()
        CommandScheduler.run()

        assertFalse(group.isScheduled())
    }

    @Test
    fun `ParallelDeadlineGroup ends when the deadline finishes, regardless of the others`() {
        val group = ParallelDeadlineGroup(finishAfter(1), finishAfter(100))

        group.schedule()
        CommandScheduler.run()

        assertFalse(group.isScheduled())
    }

    @Test
    fun `andThen composes a sequential group`() {
        val order = mutableListOf<Int>()
        val combined = finishAfter(1) { order.add(1) }.andThen(finishAfter(1) { order.add(2) })

        combined.schedule()
        CommandScheduler.run()
        CommandScheduler.run()

        assertEquals(listOf(1, 2), order)
    }
}
