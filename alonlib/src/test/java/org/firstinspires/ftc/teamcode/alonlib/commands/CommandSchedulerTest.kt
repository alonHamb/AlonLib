package org.firstinspires.ftc.teamcode.alonlib.commands

import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class CommandSchedulerTest {

    private class FakeSubsystem : SubsystemBase()

    @Before
    fun setUp() {
        CommandScheduler.reset()
        Robot.isDisabled = false
    }

    @After
    fun tearDown() {
        CommandScheduler.reset()
    }

    @Test
    fun `a scheduled command initializes, executes, and ends when finished`() {
        var initialized = false
        var executeCount = 0
        var ended: Boolean? = null
        var finished = false

        val command = FunctionalCommand({ initialized = true }, { executeCount++ }, { ended = it }, { finished })
        command.schedule()

        CommandScheduler.run()
        assertTrue(initialized)
        assertEquals(1, executeCount)
        assertTrue(command.isScheduled())

        finished = true
        CommandScheduler.run()
        assertEquals(2, executeCount)
        assertEquals(false, ended)
        assertFalse(command.isScheduled())
    }

    @Test
    fun `scheduling a command that shares a requirement interrupts the interruptible incumbent`() {
        val subsystem = FakeSubsystem()
        var firstInterrupted: Boolean? = null

        val first = FunctionalCommand({}, {}, { firstInterrupted = it }, { false }, subsystem)
        val second = FunctionalCommand({}, {}, { }, { false }, subsystem)

        first.schedule()
        CommandScheduler.run()
        assertTrue(first.isScheduled())

        second.schedule()
        assertTrue(second.isScheduled())
        assertFalse(first.isScheduled())
        assertEquals(true, firstInterrupted)
    }

    @Test
    fun `a non-interruptible command blocks a conflicting requirement until it finishes`() {
        val subsystem = FakeSubsystem()
        val first = FunctionalCommand({}, {}, {}, { false }, subsystem)
        val second = FunctionalCommand({}, {}, {}, { false }, subsystem)

        first.schedule(interruptible = false)
        CommandScheduler.run()

        second.schedule()
        assertFalse(second.isScheduled())
        assertTrue(first.isScheduled())
    }

    @Test
    fun `a subsystem's default command runs only when nothing else requires it`() {
        val subsystem = FakeSubsystem()
        var defaultRuns = 0
        val defaultCommand = RunCommand({ defaultRuns++ }, subsystem)
        subsystem.setDefaultCommand(defaultCommand)

        CommandScheduler.run()
        assertTrue(defaultCommand.isScheduled())

        // A RunCommand (not InstantCommand) so it's still running after one run() call, letting us
        // observe the "default command displaced" state before it finishes and frees the subsystem.
        var interruptingFinished = false
        val interrupting = FunctionalCommand({}, {}, {}, { interruptingFinished }, subsystem)
        interrupting.schedule()
        assertFalse(defaultCommand.isScheduled())

        CommandScheduler.run()
        assertFalse(defaultCommand.isScheduled())
        assertTrue(interrupting.isScheduled())

        interruptingFinished = true
        CommandScheduler.run()
        assertTrue(defaultCommand.isScheduled())
    }

    @Test
    fun `cancelAll ends every scheduled command as interrupted`() {
        var interrupted = false
        val command = FunctionalCommand({}, {}, { interrupted = it }, { false })
        command.schedule()
        CommandScheduler.run()

        CommandScheduler.cancelAll()

        assertTrue(interrupted)
        assertFalse(command.isScheduled())
    }
}
