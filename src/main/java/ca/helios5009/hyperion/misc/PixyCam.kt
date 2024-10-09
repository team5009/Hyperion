package ca.helios5009.hyperion.misc

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.I2cWaitControl
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.TypeConversion

@I2cDeviceType
@DeviceProperties(name = "Pixy2 Smart Vision Camera", description = "Pixy2 Smart Vision Camera", xmlTag = "Pixy2")
class PixyCam(deviceClient: I2cDeviceSynch, deviceClientIsOwned: Boolean): I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, deviceClientIsOwned) {
	override fun getManufacturer(): HardwareDevice.Manufacturer {
		return HardwareDevice.Manufacturer.ModernRobotics
	}

	override fun getDeviceName(): String {
		return "Pixy2 Smart Vision Camera"
	}

	override fun doInitialize(): Boolean {
		return true
	}

	init {

		deviceClient.i2cAddress = I2cAddr(0x54) // Pixy2 I2C address that you configure in : Settings > Pixy Paramaeters > Interface
		super.registerArmingStateCallback(false)
		deviceClient.engage()

	}

	enum class PixyRegister(private val i: Int) {
		REQUEST(0xc1ae),
		RESPONSE(0xc1af);

		fun get(): Int {
			return i
		}
	}

	enum class PixyRequestPackets(
		private val requestType: Int,
		private val request_length: Int,
		private val response_type: Int,
		private val response_length: Int,
		) {
		VERSION(	14, 0, 15, 16),
		RESOLUTION(12, 1 ,13, 2),
		CAMERA_BRIGHTNESS(16, 1, 1, 4),
		SERVOS(	18, 4, 1, 4),
		LED(	20, 3, 1, 4),
		LAMP(	22, 2, 1, 4),
		FPS(	24, 0, 1, 4),
		BLOCKS(	32, 2, 33, 14),
		MAIN_FEATURES(	48, 2, 49, 4),
		MODE(	54, 1, 1 ,4),
		NEXT_TURN(	58, 2, 1, 4),
		DEFAULT_TURN(	60, 2, 1, 4),
		VECTOR(56, 1, 1, 4),
		REVERSE_VECTOR(	62, 0, 1, 4),
		RGB(112, 5, 1, 4);

		fun getRequestType(): Int {
			return requestType
		}
		fun getRequestLength(): Int {
			return request_length
		}
		fun getResponseType(): Int {
			return response_type
		}
		fun getResponseLength(): Int {
			return response_length
		}
	}

	private fun ReadEntireWindow(readWindow: I2cDeviceSynch.ReadWindow): ByteArray? {
		deviceClient.readWindow = readWindow
		return deviceClient.read(readWindow.registerFirst, readWindow.registerCount)
	}

//	private fun requestRawVersion(): ByteArray? {
//		val versionRequest = byteArrayOf(
//			0xae.toByte(), 0xc1.toByte(), 0x0e, 0x00
//		)
//
//		versionRequest.let {
//			deviceClient.write(versionRequest)
//		}
//		deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN)
//		return ReadEntireWindow(I2cDeviceSynch.ReadWindow(PixyRegister.RESPONSE.get()))Ok
//
//	}

}



