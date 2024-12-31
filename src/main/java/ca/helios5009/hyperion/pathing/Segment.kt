package ca.helios5009.hyperion.pathing

import ca.helios5009.hyperion.misc.cosineLaw
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs

class Segment {
	private val listOfPoints: MutableList<Point> = mutableListOf()
	var lastKnownPosition = Point(0.0, 0.0).setRad(0.0) // The last point written by the User
		set(value) {
			field = value
			if (value.angleSet) {
				field.setRad(value.rot)
			}
		}
	val distanceFromTarget = AtomicReference(0.0)
	private var index = 0
		private set(value) {
			if (value < 0 || value >= length) {
				throw IndexOutOfBoundsException("The current point is out of bounds")
			}
			field = value
		}

	val current get() = listOfPoints[index]
	val last get() = listOfPoints.last()
	val first get() = listOfPoints.first()

	val length get() = listOfPoints.size

	val next: Point
		get() = run {
			if (index + 1 >= length) {
				return listOfPoints[index]
			}
			return listOfPoints[index + 1]
		}

	val previous: Point
		get() = run {
			if (index - 1 < 0) {
				return listOfPoints[index]
			}
			return listOfPoints[index - 1]
		}

	fun nextPoint() {
		index++
	}

	fun previousPoint() {
		index--
	}

	fun reset() {
		index = 0
	}

	fun hasNext() = index + 1 < length - 1

	fun hasPrevious() = index - 1 >= 0

	fun isLast() = index == length - 1

	fun isFirst() = index == 0

	operator fun get(index: Int) = listOfPoints[index]

	fun clear() {
		listOfPoints.clear()
		index = 0
	}

	fun setPath(list: List<Point>) {
		listOfPoints.clear()
		listOfPoints.addAll(list)
	}

	fun addPoint(point: Point) {
		listOfPoints.add(point)
	}

	fun removePoint(index: Int) {
		if (index < 0 || index >= length) {
			throw IndexOutOfBoundsException("The index is out of bounds")
		}
		listOfPoints.removeAt(index)
	}

	fun distanceToNext(point: Point) = point.distanceTo(current)

	fun distanceToPrevious(point: Point) = point.distanceTo(previous)

	/**
	 * Look for the next error in the path. This is used to calculate the speed factor
	 * When looking for the next error, it use's the point after the error point to control it's distance.
	 * Best practice to set point that will act as a reference point after the error if you want to control the speed sooner or later.
	 * @param point The current position of the robot
	 * @return The distance to the next error
	 *
	 * @see Point
	 */
	fun lookForNextError(point: Point): Double {
		var totalDistance = 0.0
		var currentPoint = point

		for (i in index until length) {
			totalDistance += listOfPoints[i].distanceTo(currentPoint)
			currentPoint = listOfPoints[i]
			if (i + 1 < length && currentPoint.useError) {
				val nextPoint = listOfPoints[i + 1]
				totalDistance += nextPoint.distanceTo(currentPoint)
				break
			}
		}

		return totalDistance
	}

	fun calculateTolerance(point: Point, minimumVectorTolerance: Double): Double {
		return if (current.useManualTorence) {
			current.tolerance
		} else {
			val distanceA = point.distanceTo(current)
			val distanceB = point.distanceTo(next)
			val distanceC = current.distanceTo(next)
			maxOf(minimumVectorTolerance, cosineLaw(distanceA, distanceB, distanceC))
		}
	}

	fun setHeading() {
		if (!current.angleSet) {
			current.setRad(lastKnownPosition.rot)
		}
	}

	fun setLastKnownPosition(x: Double, y: Double) {
		lastKnownPosition.x = x
		lastKnownPosition.y = y
	}

	fun setLastKnownPosition(x: Double, y: Double, rot: Double) {
		lastKnownPosition.x = x
		lastKnownPosition.y = y
		lastKnownPosition.setRad(rot)
	}

	fun setLastKnownRotation(rad: Double) {
		lastKnownPosition.setRad(rad)
	}



}