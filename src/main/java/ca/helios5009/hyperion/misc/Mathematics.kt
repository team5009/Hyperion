package ca.helios5009.hyperion.misc

import ca.helios5009.hyperion.pathing.Point
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.pow
import kotlin.math.sqrt

/** Returns -[Math.PI] <= Angle < [Math.PI]*/
fun relativeRadian(angle: Double): Double {
	val angleOut = if (angle < 0) {
		-((-angle + Math.PI) % (2 * Math.PI) - Math.PI)
	} else {
		(angle + Math.PI) % (2 * Math.PI) - Math.PI
	}
	if (angleOut >= Math.PI) return -Math.PI
	return angleOut
}

fun euclideanDistance(p1: Point, p2: Point): Double {
	return sqrt((p2.x - p1.x).pow(2.0) + (p2.y - p1.y).pow(2.0))
}

fun cubicBezier(p0: Double, c0: Double, c1: Double, p1: Double, t: Double): Double {
	return (1 - t).pow(3) * p0 + 3 * (1 - t).pow(2) * t * c0 + 3 * (1 - t) * t.pow(2) * c1 + t.pow(3) * p1
}

fun lerp(p0: Double, p1: Double, t: Double): Double {
	return p0 + t * (p1 - p0)
}

fun generateBezier(pt0: Point, ct0: Point, ct1: Point, pt1: Point): MutableList<Point> {
	val points = mutableListOf<Point>()
	val resolution = 20

	for (i in 0..resolution) {
		val t = i.toDouble() / resolution

		val x = cubicBezier(pt0.x, ct0.x, ct1.x, pt1.x, t)

		val y = cubicBezier(pt0.y, ct0.y, ct1.y, pt1.y, t)

		val calcRot = lerp(pt0.rot, pt1.rot, t);

		val rot = if (calcRot > 0)
			calcRot
		else
			calcRot + 2 * Math.PI

		when(t) {
			0.0 -> points.add(Point(x, y, pt0.event).setRad(rot))
			1.0 -> points.add(Point(x, y, pt1.event).setRad(rot))
			0.25 -> points.add(Point(x, y, ct0.event).setRad(rot))
			0.75 -> points.add(Point(x, y, ct1.event).setRad(rot))
			else -> points.add(Point(x, y).setRad(rot))
		}
	}

	return points
}

fun cosineLaw(a: Double, b: Double, c: Double): Double {
	if (a == 0.0 || b == 0.0 || c == 0.0) {
		return 0.0
	}
	return acos(((a * a) + (b * b) - (c * c)) / (2 * a * b))
}

/**
 * Various math utilities.
 */
object MathUtil {

	/**
	 * Returns the real solutions to the quadratic ax^2 + bx + c.
	 */
	@JvmStatic
	fun solveQuadratic(a: Double, b: Double, c: Double): List<Double> {
		val disc = b * b - 4 * a * c
		return when {
			disc epsilonEquals 0.0 -> listOf(-b / (2 * a))
			disc > 0.0 -> listOf(
				(-b + sqrt(disc)) / (2 * a),
				(-b - sqrt(disc)) / (2 * a)
			)
			else -> emptyList()
		}
	}
}

const val EPSILON = 1e-6

infix fun Double.epsilonEquals(other: Double) = abs(this - other) < EPSILON