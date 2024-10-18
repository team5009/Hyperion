package ca.helios5009.hyperion.misc.events

import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import java.util.concurrent.atomic.AtomicReference

/**
 * EventListener class
 *
 * This class is used to listen to events and trigger them. This is useful for multitasking
 * and to prevent the program from freezing when an event is triggered.
 *
 * @property queue The queue of events to be triggered
 * @property listeners The listeners to be called when an event is triggered
 */
class EventListener {
	private val queue = mutableListOf<String>()
	private val listeners = hashMapOf<String, MutableList<suspend () -> String?>>()

	/**
	 * Add a listener to an event to be called when the event is triggered
	 *
	 * Events starting with an underscore are ignored from the listener. This is to add optional
	 * @param event The event to listen to
	 * @param listener The listener to be called when the event is triggered (suspended)
	 *
	 */
	fun addListener(event: String, listener: suspend () -> String?) {
		if (event.startsWith("_")) {
			return
		}
		synchronized(listeners) {
			if (listeners[event] == null) {
				listeners[event] = mutableListOf()
			}
			listeners[event]!!.add(listener)
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
	 * @param event The event to trigger
	 */
	fun call(event: String) {
		if (event.startsWith("_") || event.isBlank()) {
			return
		}

		synchronized(queue) {
			queue.add(event)
			var eventRemovedFromQueue = false
			val listeners = listeners[event] ?: return
			listeners.forEach { listener ->
				if (!eventRemovedFromQueue) {
					eventRemovedFromQueue = queue.remove(event)
				}
				CoroutineScope(Dispatchers.Default).launch {
					val result = listener()
					if (result != null) {
						call(result)
					}
					return@launch
				}
			}

		}
	}

	/**
	 * Check if an event is in the queue
	 *
	 * @param event The event to check
	 * @return True if the event is in the queue, false otherwise
	 */
	fun isInQueue(event: String): Boolean {
		synchronized(queue) {
			return queue.remove(event)
		}
	}

	fun getQueueString(): String {
		synchronized(queue) {
			return queue.toString()
		}
	}
}