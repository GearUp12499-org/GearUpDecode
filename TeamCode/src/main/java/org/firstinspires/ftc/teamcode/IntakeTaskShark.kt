package org.firstinspires.ftc.teamcode

import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.*
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.BOTTOM_STOP_STOWED
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.Locks
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous

object IntakeTaskShark {
    fun intake(hw: CompBot2Hardware, timeout: Double = 0.5) = object : Group({}) {
        init {
            getScheduler()
                .add(OneShot {
                    hw.setIntakePower(0.0)
                    hw.bottomBallStop.position = BOTTOM_STOP_STOWED
                    hw.flipper.position = FLIPPER_DOWN
                    hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2)
                })
                .then(Wait.ms(250))
                .then(OneShot{
                    hw.setIntakePower(1.0)
                })
                .then(WaitUntil {
                    hw.colorTopLeft.getDistance(DistanceUnit.MM) < 95.0
                            || hw.colorTopRight.getDistance(DistanceUnit.MM) < 95.0
                })
                .then(WaitUntilContinuous(timeout) {
                    hw.frontRamp.state && hw.middleRamp.state
                })
            this.require(Locks.INTAKE_STORAGE)
        }

        override fun onFinish(completedNormally: Boolean) {
            super.onFinish(completedNormally)
            hw.setIntakePower(0.0)
            hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3)
        }
    }
    fun intakeAfter(hw: CompBot2Hardware) = object: Group({}) {
        init {
            getScheduler()
                .add(OneShot {
                    hw.setIntakePower(0.0)
                })
                .then(Wait.s(0.05))
                .then(OneShot {
                    hw.shooterBallStop.position = SHOOTER_STOP_UP
                })
                .then(Wait.s(0.15))
        }
    }
}