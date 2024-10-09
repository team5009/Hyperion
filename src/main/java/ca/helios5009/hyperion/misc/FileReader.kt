package ca.helios5009.hyperion.misc

import org.firstinspires.ftc.robotcore.internal.system.AppUtil

import kotlin.io.path.Path


class FileReader {
	companion object {
		fun getFile(path: String): String {
			return AppUtil.FIRST_FOLDER.absolutePath + "/Hyperion/$path"
		}
	}
}