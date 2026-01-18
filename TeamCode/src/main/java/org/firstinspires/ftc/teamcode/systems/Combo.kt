package org.firstinspires.ftc.teamcode.systems

import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous
import org.firstinspires.ftc.teamcode.utilities.StaticStore

object Combo {
    fun intake(hw: CompBot2Hardware, power: Double = CompBot2Hardware.INTAKE_POWER) = object : Group({}) {
        init {
            getScheduler()
                .add(OneShot {
                    hw.intake.power = 0.0
                    hw.bottomBallStop.position =
                        if (hw.colorTopLeft.getDistance(DistanceUnit.MM) < 100.0) CompBot2Hardware.BOTTOM_BALL_STOP
                        else CompBot2Hardware.BOTTOM_STOP_STOWED
                    hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2)
                })
                .then(Wait.ms(250))
                .then(OneShot {
                    hw.intake.power = power
                })
                .then(WaitUntil {
                    hw.colorTopLeft.getDistance(DistanceUnit.MM) < 100.0
                })
                .then(OneShot {
                    hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_BALL_STOP
                })
                .then(WaitUntilContinuous(.5) {
                    hw.frontRamp.state && hw.middleRamp.state
                })
            this.require(CompBot2Hardware.Locks.INTAKE_STORAGE)
        }

        override fun onFinish(completedNormally: Boolean) {
            super.onFinish(completedNormally)
            hw.intake.power = 0.0
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
            hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3)
        }
    }

    @JvmOverloads
    fun shoot(hw: CompBot2Hardware, shooter: ShooterImpl, flipperWait: Double = 0.3) = object : Group({}) {
        init {
            getScheduler()
                .add(OneShot {
                    hw.intake.power = 1.0
                    hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                })
                .then(WaitUntilContinuous(flipperWait) {
                    !hw.frontRamp.state && (hw.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
                            || hw.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)
                })
                .then(OneShot {
                    hw.flipper.position = CompBot2Hardware.FLIPPER_UP
                })
                .then(Wait.ms(1000))
                .then(OneShot {
                    hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
                    hw.intake.power = CompBot2Hardware.OUTTAKE_POWER
                })
                .then(Wait.ms(500))
            this.require(CompBot2Hardware.Locks.INTAKE_STORAGE)
        }

        override fun onFinish(completedNormally: Boolean) {
            super.onFinish(completedNormally)
            shooter.setTarget(0.0)
            hw.intake.power = 0.0
            hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
        }
    }
}