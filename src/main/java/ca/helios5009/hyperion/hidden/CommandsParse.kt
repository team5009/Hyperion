package ca.helios5009.hyperion.hidden

import android.annotation.SuppressLint
import ca.helios5009.hyperion.misc.cubicBezier
import ca.helios5009.hyperion.misc.lerp
import ca.helios5009.hyperion.pathing.Point
import com.qualcomm.robotcore.util.RobotLog

@SuppressLint("UnsafeDynamicallyLoadedCode")
class CommandsParse {
	init {
		RobotLog.vv("Hyperion", "Loading Hyperion Library")
//		RobotLog.vv("Hyperion", System.getProperty("os.name"))
//		val library = FileReader().getFile("libhyperion.so")
		try {
//			System.load(library)
			System.loadLibrary("hyperion")
			RobotLog.vv("Hyperion", "Hyperion Library Loaded")
		} catch (e: UnsatisfiedLinkError) {
			RobotLog.ee("Hyperion", "Error loading Hyperion Library: $e")
		}
//		System.loadLibrary("hyperion")
	}

	external fun read(commands: String): String

	fun bezier(pt0: Point, ct0: Point, ct1: Point, pt1: Point): MutableList<Point> {
		val points = mutableListOf<Point>()
		val resolution = 20

		for (i in 0..resolution) {
			val t = i.toDouble() / resolution

			val x = cubicBezier(pt0.x, ct0.x, ct1.x, pt1.x, t)

			val y = cubicBezier(pt0.y, ct0.y, ct1.y, pt1.y, t)

			val rot = if (lerp(pt0.rot, pt1.rot, t) > 0)
				lerp(pt0.rot, pt1.rot, t)
			else
				lerp(pt0.rot, pt1.rot, t) + 360

			when(t) {
				0.0 -> points.add(Point(x, y, 0.0, pt0.event))
				1.0 -> points.add(Point(x, y, 0.0, pt1.event))
				0.25 -> points.add(Point(x, y, 0.0, ct0.event))
				0.75 -> points.add(Point(x, y, 0.0, ct1.event))
				else -> points.add(Point(x, y, 0.0))
			}
		}

		return points
	}

}