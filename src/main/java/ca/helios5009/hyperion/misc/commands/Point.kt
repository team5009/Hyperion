package ca.helios5009.hyperion.misc.commands

import ca.helios5009.hyperion.misc.relativeRadian

/**
 * A point in a path
 * @param x The x coordinate of the point
 * @param y The y coordinate of the point
 * @param rot The rotation of the robot at the point in degrees
 * @param event The event to call at the point
 */
class Point(var x: Double, var y: Double, rot: Double, val event: EventCall = EventCall("_")) {
	var rot = relativeRadian(rot * Math.PI / 180)
	var tolerance = 1.0
	var type = PointType.Global
	var useError = false
	var useManualTorence = false
	/**
	 * Sets the point to be a local point.
	 * Local points are relative to the current position of the robot
	 * @return The point
	 */
	fun setLocal():Point {
		type = PointType.Relative
		return this
	}

	/**
	 * Sets the point to use error.
	 * Error is the distance between the target and the actual position of the robot
	 * This is useful for PID control. Use mostly in continuous movements where you want to force the bot to go slower in certain situations.
	 * @return The point
	 */
	fun useError():Point {
		useError = true
		return this
	}

	fun setTolerance(tolerence: Double = 1.0): Point {
		this.tolerance = tolerence
		useManualTorence = true
		return this
	}

	fun clone(): Point {
		return Point(x, y, rot * 180/Math.PI, event)
	}

	override fun equals(other: Any?): Boolean {
		if (other is Point) {
			return x == other.x && y == other.y && rot == other.rot
		}
		return false
	}
}

enum class PointType {
	Global, Relative
}