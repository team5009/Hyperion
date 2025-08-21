package ca.helios5009.hyperion.hardware

import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.pathing.Point
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

class Pinpoint(
    hardwareMap: HardwareMap,
    name: String
): Odometry {
    val pinpoint = hardwareMap.get(name) as GoBildaPinpointDriver
    var distanceUnit: DistanceUnit = DistanceUnit.INCH
    override var position: Point = Point(0.0, 0.0).setRad(0.0)
        get() = run {
            pinpoint.update()
            val position = pinpoint.position
            field.set(position.getX(distanceUnit), position.getY(distanceUnit), position.getHeading(AngleUnit.RADIANS))
            field
        }
        set(value) {
            pinpoint.position = Pose2D(distanceUnit, value.x, value.y, AngleUnit.RADIANS, value.rot)
            field = value
        }

    val deviceStatus get() = pinpoint.deviceStatus

    var offset : Point
        get() = Point(pinpoint.getXOffset(distanceUnit).toDouble(), pinpoint.getYOffset(distanceUnit).toDouble())
        set(value) { pinpoint.setOffsets(value.x, value.y, distanceUnit) }

    fun setEncoderResolution(res: Double) {
        pinpoint.setEncoderResolution(res, distanceUnit)
    }

    fun switchEncoderDirection(x: Boolean, y: Boolean) {
        pinpoint.setEncoderDirections(
            if (x) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD,
            if (y) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD)
    }

    fun recalibrateIMU() {
        pinpoint.recalibrateIMU()
    }

    init {
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.setOffsets(0.0, 0.0, distanceUnit)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD) // Default encoder resolution
    }

}