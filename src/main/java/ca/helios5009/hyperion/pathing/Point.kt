package ca.helios5009.hyperion.pathing

import android.annotation.SuppressLint
import ca.helios5009.hyperion.misc.relativeRadian
import java.util.Objects
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * A point in a path
 * @param x The x coordinate of the point
 * @param y The y coordinate of the point
 * @param event The event to call at the point
 */
class Point(var x: Double, var y: Double, val event: String = "_") {
	var rot = 0.0
		set(value) {
			field = relativeRadian(value)
		}

	var tolerance = 1.0
		get() = if (useManualTorence) field else 1.0
		set(value) {
			if (value < 0) {
				throw IllegalArgumentException("Tolerance cannot be less than 0")
			}
			field = value
			useManualTorence = true
		}

	var type = PointType.Global
		private set
	var useError = false
		private set
	var useManualTorence = false
		private set
	var angleSet = false
		private set
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

	/**
	 * Set the angle of the point in radians
	 * @param rot The angle in radians
	 * @return The point
	 */
	fun setRad(rot: Double): Point {
		this.rot = rot
		angleSet = true
		return this
	}

	/**
	 * Set the angle of the point in degrees
	 * @param rot The angle in degrees
	 * @return The point
	 */
	fun setDeg(rot: Double): Point {
		this.rot = rot * Math.PI / 180
		angleSet = true
		return this
	}

	fun clone(): Point {
		return Point(x, y, event).setRad(rot)
	}

	@SuppressLint("DefaultLocale")
	override fun toString() = String.format("x: %.2f, y: %.2f, rot: %.2f", x, y, rot)

	override fun equals(other: Any?): Boolean {
		if (other is Point) {
			return x == other.x && y == other.y && rot == other.rot
		}
		return false
	}
	operator fun plus(point: Point): Point {
		return Point(x + point.x, y + point.y, event).setRad(rot + point.rot)
	}
	operator fun minus(point: Point): Point {
		return Point(x - point.x, y - point.y, event).setRad(rot - point.rot)
	}
	operator fun times(point: Point): Point {
		return Point(x * point.x, y * point.y, event).setRad(rot * point.rot)
	}
	operator fun div(point: Point): Point {
		return Point(x / point.x, y / point.y, event).setRad(rot / point.rot)
	}
	operator fun plusAssign(point: Point) {
		x += point.x
		y += point.y
		rot += point.rot
	}
	operator fun minusAssign(point: Point) {
		x -= point.x
		y -= point.y
		rot -= point.rot
	}
	operator fun timesAssign(point: Point) {
		x *= point.x
		y *= point.y
		rot *= point.rot
	}
	operator fun divAssign(point: Point) {
		x /= point.x
		y /= point.y
		rot /= point.rot
	}

	fun distanceTo(point: Point): Double {
		return sqrt((point.x - x).pow(2.0) + (point.y - y).pow(2.0))
	}

	override fun hashCode() = Objects.hash(x, y, rot)


}

enum class PointType {
	Global, Relative
}