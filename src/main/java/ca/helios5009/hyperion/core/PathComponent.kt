package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.misc.commands.Point

class PathComponent {

	private val path: List<Point> = listOf()

	fun getPath(): List<Point> {
		return path
	}

	fun addPoint(point: Point) {
		path.plus(point)
	}

	fun removePoint(point: Point) {
		path.minus(point)
	}
}