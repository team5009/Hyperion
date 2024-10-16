package ca.helios5009.hyperion.pathing

import android.annotation.SuppressLint
import ca.helios5009.hyperion.misc.relativeRadian
import java.util.Objects

/**
 * A point in a path
 * @param x The x coordinate of the point
 * @param y The y coordinate of the point
 * @param rot The rotation of the robot at the point in degrees
 * @param event The event to call at the point
 */
class Point(var x: Double, var y: Double, rot: Double, val event: String = "_") {
	var rot = relativeRadian(rot * Math.PI / 180)
	private var tolerance = 1.0
	var type = PointType.Global
	var useError = false
	var useManualTorence = false
	/**
	 * Sets the point to be a local point.
	 * Local points are relative to the current position of the robot
	 * @return The point
	 */
	@Deprecated("Breaks segment algorithm")
	fun setLocal(): Point {
		type = PointType.Relative
		return this
	}

	/**
	 * Sets the point to use error.
	 * Error is the distance between the target and the actual position of the robot
	 * This is useful for PID control. Use mostly in continuous movements where you want to force the bot to go slower in certain situations.
	 * @return The point
	 */
	fun useError(): Point {
		useError = true
		return this
	}

	fun setTolerance(tolerence: Double = 1.0): Point {
		if (tolerence < 0) {
			throw IllegalArgumentException("Tolerance cannot be less than 0")
		}
		this.tolerance = tolerence
		useManualTorence = true
		return this
	}

	fun getTolerance() = tolerance

	fun clone(): Point {
		return Point(x, y, rot * 180/Math.PI, event)
	}


	@SuppressLint("DefaultLocale")
	override fun toString() = String.format("x: %.2f, y: %.2f, rot: %.2f", x, y, rot)

	override fun equals(other: Any?): Boolean {
		if (other is Point) {
			return x == other.x && y == other.y && rot == other.rot
		}
		return false
	}

	override fun hashCode() = Objects.hash(x, y, rot)

}



enum class PointType {
	Global, Relative
}