package ca.helios5009.hyperion.misc

import ca.helios5009.hyperion.pathing.Point

interface Odometry {
	/*The current position of the robot*/
	var position: Point
}