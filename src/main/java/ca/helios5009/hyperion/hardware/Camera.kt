package ca.helios5009.hyperion.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class Camera(private val hardware: HardwareMap, private val optimized: Boolean = false, private val name: String = "Webcam 1") {
	private val builder = VisionPortal.Builder()

	init {
		builder.setCamera(hardware.get(WebcamName::class.java, name))
		if (!optimized) {
			builder.enableLiveView(true) //False for more performance
			builder.setStreamFormat(VisionPortal.StreamFormat.YUY2) //YUY2 gives the best performance when streaming is enabled
		}
	}

	fun initAprilTag(): AprilTagProcessor {
		val processor = AprilTagProcessor.Builder()
		if (!optimized) {
			processor.setDrawAxes(true) //False for more performance
			processor.setDrawTagOutline(true) //False for more performance
			processor.setDrawCubeProjection(true) //False for more performance
		} else {
			processor.setDrawAxes(false) //False for more performance
			processor.setDrawTagOutline(false) //False for more performance
			processor.setDrawCubeProjection(false) //False for more performance
		}
		processor.setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
		builder.addProcessor(processor.build())
		return processor.build()
	}

	fun addProcessor(processor: VisionProcessor): Camera {
		builder.addProcessor(processor)
		return this
	}

	fun build(): VisionPortal {
		return builder.build()
	}
}