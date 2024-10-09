package ca.helios5009.hyperion.core


import ca.helios5009.hyperion.misc.FileReader
import java.io.File

class AutonPaths {
	val folder = FileReader.getFile("autonomous")
	fun storePath(content: String, fileName: String) {
		File("${folder}/${fileName}").writeText(content)
	}

	fun readPath(fileName: String): String {
		return File("${folder}/${fileName}").readText()
	}
	fun getAllPaths(): List<String> {
		return File(folder).listFiles()!!.map { it.name }
	}
}