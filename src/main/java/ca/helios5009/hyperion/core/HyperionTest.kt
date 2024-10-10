package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.pathing.Point
import ca.helios5009.hyperion.misc.FileReader
import ca.helios5009.hyperion.pathing.PathBuilder
import java.io.File

/**
 * HyperionTest is a class that reads a file and runs the commands in the file.
 * You can create a path through the Hyperion Controller app.
 *
 * Given a path, the HyperionTest will read the file and run the commands 1 by 1.
 *
 * @param movement The Movement object that is used to move the robot.
 * @param path The path to the file that contains the commands.
 * @constructor Create a new HyperionTest object.
 *
 * @see Movement
 * @see FileReader
 */
class HyperionTest(movement: Movement, private val path: String) {
	private val HyperionPath = PathBuilder(movement)

	fun run() {
		val file = File(path)
		var isSegment = false
		var segmentLength: Byte = 0
		var currentSegmentPoint: Byte = 0
		val currentSegment = mutableListOf<Point>()

		for (line in file.readLines()) {
			if (line.startsWith('-')) {
				val stringCommand = line.substring(1)
				val splitCommand = stringCommand.split(" ").toMutableList()
				val command = splitCommand.removeFirst()
				val args = splitCommand.removeFirst()
				if (command == "end") {
					HyperionPath.end(args)
					break;
				}

				when (command) {
					"start" -> {
						val eventCall = splitCommand.removeFirst()
						val pointSplit = args.split(",")
						val point = Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), pointSplit[2].toDouble(), eventCall)
						HyperionPath.start(point)
					}
					"segment" -> {
						isSegment = true
						segmentLength = args.toByte()
					}
					"wait" -> {
						val eventCall = splitCommand.removeFirst()
						if (args[0].isDigit()) {
							HyperionPath.wait(convertTime(args), eventCall)
						} else {
							HyperionPath.wait(args, eventCall)
						}
					}
					else -> {
						throw Exception("Invalid command")
					}
				}
			} else if (isSegment) {
				if (segmentLength == 0.toByte()) {
					throw Exception("Invalid segment length")
				}
				val splitCommand = line.split(" ").toMutableList()
				var pointString = splitCommand.removeFirst()
				val tolerance = splitCommand.removeFirst()
				val eventCall = splitCommand.removeFirst()
				var usingError = false

				pointString = if (pointString.startsWith("*")) {
					usingError = true
					pointString.replace("*", "")
				} else {
					pointString
				}

				val pointSplit = pointString.split(",").toMutableList()
				val point = if (usingError) {
					Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), pointSplit[2].toDouble(), eventCall).useError()
				} else {
					Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), pointSplit[2].toDouble(), eventCall)
				}

				if (!tolerance.startsWith('_') && tolerance != "0") {
					point.setTolerance(tolerance.toDouble())
				}
				currentSegment.add(point)
				currentSegmentPoint++
				if (currentSegmentPoint == segmentLength) {
					HyperionPath.segment(currentSegment)
					currentSegmentPoint = 0
					currentSegment.clear()
					isSegment = false
				}
			}
		}
	}

	private fun convertTime(time: String): Double {
		var suffix = ""
		var timeString = ""
		for (char in time) {
			if (char.isDigit() || char == '.') {
				timeString += char
			} else {
				suffix += char
			}
		}

		return when (suffix) {
			"ms" -> {
				timeString.toDouble()
			}
			"s" -> {
				timeString.toDouble() * 1000
			}
			"m" -> {
				timeString.toDouble() * 60000
			}
			else -> {
				throw Exception("Invalid time")
			}
		}
	}
}