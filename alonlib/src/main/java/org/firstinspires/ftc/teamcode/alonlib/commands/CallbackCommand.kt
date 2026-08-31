package org.firstinspires.ftc.teamcode.alonlib.commands

/** Wraps [command] (scheduling it on init) so one-off callbacks can be attached to it, each firing once and then removing itself. */
class CallbackCommand<T : Command>(private val command: T) : Command {

	private val whenRunnables = mutableMapOf<() -> Boolean, () -> Unit>()
	private val whenCommands = mutableMapOf<() -> Boolean, Command>()
	private val whenConsumers = mutableMapOf<() -> Boolean, (T) -> Unit>()
	private val whenSelfRunnables = mutableMapOf<(T) -> Boolean, () -> Unit>()
	private val whenSelfCommands = mutableMapOf<(T) -> Boolean, Command>()
	private val whenSelfConsumers = mutableMapOf<(T) -> Boolean, (T) -> Unit>()

	private val _requirements = mutableSetOf<Subsystem>()
	override val requirement: Set<Subsystem> get() = _requirements

	fun addRequirements(vararg requirements: Subsystem) = apply { _requirements.addAll(requirements) }

	/** Runs [action] the first time [condition] becomes true. */
	fun whenTrue(condition: () -> Boolean, action: () -> Unit) = apply { whenRunnables[condition] = action }

	/** Schedules [action] the first time [condition] becomes true. */
	fun whenTrue(condition: () -> Boolean, action: Command) = apply { whenCommands[condition] = action }

	/** Calls [action] with the wrapped command the first time [condition] becomes true. */
	fun whenTrueSelf(condition: () -> Boolean, action: (T) -> Unit) = apply { whenConsumers[condition] = action }

	/** Runs [action] the first time [condition] (tested against the wrapped command) becomes true. */
	fun whenSelf(condition: (T) -> Boolean, action: () -> Unit) = apply { whenSelfRunnables[condition] = action }

	/** Schedules [action] the first time [condition] (tested against the wrapped command) becomes true. */
	fun whenSelf(condition: (T) -> Boolean, action: Command) = apply { whenSelfCommands[condition] = action }

	/** Calls [action] with the wrapped command the first time [condition] (tested against it) becomes true. */
	fun whenSelf(condition: (T) -> Boolean, action: (T) -> Unit) = apply { whenSelfConsumers[condition] = action }

	override fun initialize() = command.schedule()

	override fun execute() {
		fireDue(whenRunnables) { it() }
		fireDue(whenCommands) { it.schedule() }
		fireDue(whenConsumers) { it(command) }
		fireDueSelf(whenSelfRunnables) { it() }
		fireDueSelf(whenSelfCommands) { it.schedule() }
		fireDueSelf(whenSelfConsumers) { it(command) }
	}

	private fun <A> fireDue(map: MutableMap<() -> Boolean, A>, fire: (A) -> Unit) {
		val iterator = map.entries.iterator()
		while (iterator.hasNext()) {
			val (condition, action) = iterator.next()
			if (condition()) {
				fire(action)
				iterator.remove()
			}
		}
	}

	private fun <A> fireDueSelf(map: MutableMap<(T) -> Boolean, A>, fire: (A) -> Unit) {
		val iterator = map.entries.iterator()
		while (iterator.hasNext()) {
			val (condition, action) = iterator.next()
			if (condition(command)) {
				fire(action)
				iterator.remove()
			}
		}
	}

	override fun isFinished() = !CommandScheduler.isScheduled(command)
}
