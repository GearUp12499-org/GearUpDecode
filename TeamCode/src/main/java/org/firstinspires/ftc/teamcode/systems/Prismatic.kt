package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver
import org.firstinspires.ftc.teamcode.drivers.IGoBildaPrismDriver
import org.firstinspires.ftc.teamcode.drivers.PrismAnimations
import org.firstinspires.ftc.teamcode.drivers.PrismAnimations.AnimationBase
import org.firstinspires.ftc.teamcode.drivers.PrismColor

object Prismatic {
    object NoAnimation : AnimationBase(PrismAnimations.AnimationType.NONE) {
        override fun updateAnimationOverI2C(
            deviceClient: I2cDeviceSynchSimple,
            isInsertingAnimation: Boolean
        ) {
            if (isInsertingAnimation)
                deviceClient.write(
                    layerHeight.register.address,
                    GetByteArray(0, animationType.AnimationTypeIndex)
                )
            deviceClient.write(layerHeight.register.address, GetByteArray(1, brightness))
            deviceClient.write(layerHeight.register.address, GetByteArray(2, startIndex))
            deviceClient.write(layerHeight.register.address, GetByteArray(3, stopIndex))
        }

        override fun updateAnimationSpecificValuesOverI2C(deviceClient: I2cDeviceSynchSimple?) {
        }
    }

    const val SEG1A = 3
    const val SEG1B = 11
    const val SEG2A = 21
    const val SEG2B = 29

    val purple1 = PrismAnimations.Solid(PrismColor.PURPLE)
        .also {
            it.setIndexes(SEG1A, SEG1B)
            it.brightness = 100
        }
    val purple2 = PrismAnimations.Solid(PrismColor.PURPLE)
        .also {
            it.setIndexes(SEG2A, SEG2B)
            it.brightness = 100
        }
    val green1 = PrismAnimations.Solid(PrismColor.GREEN)
        .also {
            it.setIndexes(SEG1A, SEG1B)
            it.brightness = 100
        }
    val green2 = PrismAnimations.Solid(PrismColor.GREEN)
        .also {
            it.setIndexes(SEG2A, SEG2B)
            it.brightness = 100
        }

    val redColor = PrismAnimations.Pulse(PrismColor(255, 0, 0), PrismColor(128, 0, 0), 1000)
        .also {
            it.brightness = 50
        }
    val blueColor = PrismAnimations.Pulse(PrismColor(0, 0, 255), PrismColor(0, 0, 128), 1000)
        .also {
            it.brightness = 50
        }

    enum class Mode {
        MAIN,
        ALTERNATE
    }

    fun configurationLights(
        prism: IGoBildaPrismDriver,
        red: Boolean,
        mode: Mode
    ) {
        prism.clearAllAnimations()

        val color = if (red) redColor else blueColor

        val first = when (mode) {
            Mode.MAIN -> purple1
            Mode.ALTERNATE -> green1
        }
        val second = when (mode) {
            Mode.MAIN -> purple2
            Mode.ALTERNATE -> green2
        }
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, color)
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, first)
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, second)
    }
}