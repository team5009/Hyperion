package ca.helios5009.hyperion.misc

import ca.helios5009.hyperion.pathing.Point

interface Odometry {

	fun getPosition(): Point
	fun setPosition(point: Point)
}