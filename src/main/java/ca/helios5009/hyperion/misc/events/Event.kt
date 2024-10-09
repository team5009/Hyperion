package ca.helios5009.hyperion.misc.events

abstract class Event(val event: String) {

	abstract suspend fun run()
}