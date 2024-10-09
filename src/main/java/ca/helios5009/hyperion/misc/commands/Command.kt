package ca.helios5009.hyperion.misc.commands

import com.google.gson.annotations.SerializedName

class Command() {

}
enum class CommandType(s: String) {

	@SerializedName("Start") START("Start"),
	@SerializedName("End") END("End"),
	@SerializedName("Line") LINE("Line"),
	@SerializedName("Bezier") BEZIER("Bezier"),
	@SerializedName("Spline") SPLINE("Spline"),
	@SerializedName("Wait") WAIT("Wait")

}


