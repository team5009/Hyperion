package ca.helios5009.hyperion.logging

import ca.helios5009.hyperion.misc.FileReader
import java.io.File
import java.util.Date

class Logger {
	companion object {
		val loggingDirPath = FileReader.getHyperionPath("logs")
		val botPositionsPath = FileReader.getHyperionPath("logs/location_tracking")
		var previousLogFileTime = Date()
	}
	private val botPositions = File(botPositionsPath)
	private lateinit var currentLogFile: File

	init {
		// Check if logging is made
		if (!botPositions.exists()) {
			botPositions.mkdirs()
		}
		currentLogFile = File("$loggingDirPath/current.log")
		if (currentLogFile.exists()) {
			currentLogFile.renameTo(File("$loggingDirPath/${previousLogFileTime}.log"))
			currentLogFile = File("$loggingDirPath/current.log")
		}


	}

}