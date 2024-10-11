package ca.helios5009.hyperion.misc.events

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import java.util.concurrent.atomic.AtomicReference

/**
 * EventListener is a class that allows creation of non-blocking events that can be triggered at user-defined points.
 * This is useful for multitasking and running multiple tasks at once.
 *
 * @see Event
 */
class EventListener() {
	var value = AtomicReference("")
	private val triggerFunctions = mutableListOf<Event>()
	private val queue = AtomicReference(mutableListOf<String>())
	private val scopes = mutableListOf<Job>()

	/**
	 * Subscribe to an event
	 * @param callbackClass The class that will be called when the event is triggered\
	 * @see Event
	 */
	fun subscribe(vararg callbackClass: Event) {
		callbackClass.forEach {
			triggerFunctions.add(it)
		}
	}

	/**
	 * Trigger an event. Events starting with an underscore are ignored and not triggered
	 *
	 * All events are ran in a coroutine scope. This is to prevent the program from freezing
	 * when an event is triggered. This is useful for multitasking.
	 *
	 * The event called is added to a queue. This is so that the event don't overlap each other.
	 *
	 * @param newValue The event to trigger
	 * @see Event
	 */
	fun call(newValue: String) {
		if (newValue.startsWith('_')) {
			return
		}
		queue.get().add(newValue)
		var deleted = false
		triggerFunctions.forEach {
			if (it.event.lowercase() == newValue) {
				if (!deleted) {
					deleted = queue.get().remove(newValue)
				}
				scopes.add(CoroutineScope(Dispatchers.Default).launch {
					it.run()
					return@launch
				})
			}
		}
	}

	fun isInQueue(message: String): Boolean {
		val queue = queue.get()
		if (queue.isEmpty()) {
			return false
		}

		if (queue.contains(message)) {
			queue.remove(message)
			return true
		}
		return false
	}

	fun clearQueue() {
		queue.get().clear()
	}

	fun clearScopes() {
		scopes.forEach {
			it.cancel()
		}
		scopes.clear()
	}

}
