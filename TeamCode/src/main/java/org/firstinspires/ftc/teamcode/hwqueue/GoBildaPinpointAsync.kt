package org.firstinspires.ftc.teamcode.hwqueue

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.ByteOrder.LITTLE_ENDIAN
import java.util.concurrent.Future
import kotlin.math.abs

class GoBildaPinpointAsync(val underlying: GoBildaPinpointDriver) {
    private val sink by LogicThread.hw

    var deviceStatus = 0; private set
    var loopTime = 0; private set
    var xEncoderValue = 0; private set
    var yEncoderValue = 0; private set
    var xPosition = 0f; private set
    var yPosition = 0f; private set
    var hOrientation = 0f; private set
    var xVelocity = 0f; private set
    var yVelocity = 0f; private set
    var hVelocity = 0f; private set

    companion object {
        // registers
        const val BULK_READ = 18
        const val POSITION_THRESHOLD = 5000.0        // more than one FTC field in mm
        const val HEADING_THRESHOLD = 120.0          // About 20 full rotations in Radians
        const val VELOCITY_THRESHOLD = 10000.0       // 10k mm/sec is faster than an FTC robot should be going...
        const val HEADING_VELOCITY_THRESHOLD = 120.0  // About 20 rotations per second

        private fun byteArrayToFloat(byteArr: ByteArray, order: ByteOrder) =
            ByteBuffer.wrap(byteArr).order(order).getFloat()
    }

    private fun validatePosition(previous: Float, current: Float, thresh: Double, useTiming: Boolean): Float {
        val isMissing = useTiming && (loopTime < 1)
        val isBroken = isMissing || current.isNaN() || abs(current - previous) > thresh
        if (!isBroken) return current
        deviceStatus = 1 shl 5 // DeviceStatus.FAULT_BAD_READ
        return previous
    }

    private fun validateVelocity(previous: Float, current: Float, thresh: Double): Float {
        val isBroken = current.isNaN() || abs(current) > thresh
        if (!isBroken) return current
        deviceStatus = 1 shl 5
        return previous
    }

    private var fut: Future<ByteArray>? = null

    private fun updateFromFuture() {
        val oldPosX = xPosition
        val oldPosY = yPosition
        val oldPosH = hOrientation
        val oldVelX = xVelocity
        val oldVelY = yVelocity
        val oldVelH = hVelocity

        val bArr = fut!!.get()!!
        deviceStatus =
            byteArrayToInt(bArr.copyOfRange(0, 4), LITTLE_ENDIAN)
        loopTime =
            byteArrayToInt(bArr.copyOfRange(4, 8), LITTLE_ENDIAN)
        xEncoderValue =
            byteArrayToInt(bArr.copyOfRange(8, 12), LITTLE_ENDIAN)
        yEncoderValue =
            byteArrayToInt(bArr.copyOfRange(12, 16), LITTLE_ENDIAN)
        xPosition = byteArrayToFloat(bArr.copyOfRange(16, 20), LITTLE_ENDIAN)
        yPosition = byteArrayToFloat(bArr.copyOfRange(20, 24), LITTLE_ENDIAN)
        hOrientation = byteArrayToFloat(bArr.copyOfRange(24, 28), LITTLE_ENDIAN)
        xVelocity = byteArrayToFloat(bArr.copyOfRange(28, 32), LITTLE_ENDIAN)
        yVelocity = byteArrayToFloat(bArr.copyOfRange(32, 36), LITTLE_ENDIAN)
        hVelocity = byteArrayToFloat(bArr.copyOfRange(36, 40), LITTLE_ENDIAN)

        xPosition = validatePosition(oldPosX, xPosition, POSITION_THRESHOLD, true)
        yPosition = validatePosition(oldPosY, yPosition, POSITION_THRESHOLD, true)
        hOrientation = validatePosition(oldPosH, hOrientation, HEADING_THRESHOLD, true)
        xVelocity = validateVelocity(oldVelX, xVelocity, VELOCITY_THRESHOLD)
        yVelocity = validateVelocity(oldVelY, yVelocity, VELOCITY_THRESHOLD)
        hVelocity = validateVelocity(oldVelH, hVelocity, HEADING_VELOCITY_THRESHOLD)
    }

    fun update() {
        fut?.let {
            if (!it.isDone) return
            updateFromFuture()
        }
        fut = sink.supply(object : HardwareQuery<ByteArray>() {
            override fun runInner(): ByteArray {
                return underlying.deviceClient.read(BULK_READ, 40)
            }
        })
    }
}