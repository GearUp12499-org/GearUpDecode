package org.firstinspires.ftc.teamcode.systems

import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.*
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous
import org.firstinspires.ftc.teamcode.utilities.StaticStore

object Combo {
    fun intake(hw: CompBot2Hardware, power: Double = INTAKE_POWER, timeout: Double = 0.5) = object : Group({}) {
        init {
            getScheduler()
                .add(OneShot {
                    hw.setIntakePower(0.0)
                    hw.bottomBallStop.position = BOTTOM_STOP_STOWED
//                        if (hw.colorTopLeft.getDistance(DistanceUnit.MM) < 100.0) BOTTOM_BALL_STOP
//                        else BOTTOM_STOP_STOWED
                    hw.flipper.position = FLIPPER_DOWN
                    hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2)
                })
                .then(Wait.ms(250))
                .then(OneShot {
                    hw.setIntakePower(power)
                })
                .then(WaitUntil {
                    hw.colorTopLeft.getDistance(DistanceUnit.MM) < 100.0
                })
                .then(OneShot {
//                    hw.bottomBallStop.position = BOTTOM_BALL_STOP
                })
                .then(WaitUntilContinuous(timeout) {
                    hw.frontRamp.state && hw.middleRamp.state
                })
            this.require(Locks.INTAKE_STORAGE)
        }

        override fun onFinish(completedNormally: Boolean) {
            super.onFinish(completedNormally)
            hw.setIntakePower(0.0)
//            hw.bottomBallStop.position = BOTTOM_STOP_STOWED
            hw.shooterBallStop.position = SHOOTER_STOP_UP
            hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3)
        }
    }

    fun intakeBox(hw: CompBot2Hardware, power: Double = INTAKE_POWER) = object : Group({}) {
        init {
            getScheduler()
                .add(OneShot {
                    hw.setIntakePower(0.0)
//                    hw.ballStop.position = BALL_STOP_MIDDLE
                    hw.slider.position = SLIDER_IN
                    hw.flipper.position = FLIPPER_DOWN
                    hw.bottomBallStop.position = BOTTOM_STOP_OUT
                    hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2)
                })
                .then(Wait.ms(250))
                .then(OneShot {
                    hw.setIntakePower(power)
                })
                .then(WaitUntil {
                    hw.middleRamp.state
                })
                .then(Wait.s(0.60))
                .then(OneShot {
                    hw.slider.position = SLIDER_MIDDLE
//                    hw.ballStop.position = BALL_STOP_STOWED
                })
                .then(WaitUntilContinuous(.5) {
                    hw.frontRamp.state && hw.middleRamp.state
                })
            this.require(Locks.INTAKE_STORAGE)
        }

        override fun onFinish(completedNormally: Boolean) {
            super.onFinish(completedNormally)
            hw.setIntakePower(0.0)
            hw.slider.position = SLIDER_OUT
            hw.shooterBallStop.position = SHOOTER_STOP_UP
            hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3)
        }
    }

    @JvmOverloads
    fun shootBoxLast(hw: CompBot2Hardware, shooter: ShooterImpl, flipperWait: Double = 0.3) =
        object : Group({}) {
            init {
                getScheduler()
                    .add(OneShot {
                        hw.setIntakePower(1.0)
                        hw.slider.position = SLIDER_OUT
                        hw.shooterBallStop.position = SHOOTER_STOP_UP
                        hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                    })
                    .then(WaitUntilContinuous(flipperWait) {
                        !hw.frontRamp.state && (hw.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
                                || hw.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)
                    })
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_UP
                    })
                    .then(Wait.ms(1000))
                    .then(OneShot {
                        hw.slider.position = SLIDER_IN
                        hw.flipper.position = FLIPPER_DOWN
                    })
                    .then(Wait.ms(1000))
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_UP
                    })
                    .then(Wait.ms(500))
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_DOWN
                        hw.setIntakePower(OUTTAKE_POWER)
                    })
                    .then(Wait.ms(500))
            }

            override fun onFinish(completedNormally: Boolean) {
                super.onFinish(completedNormally)
//                shooter.setTarget(0.0)
                hw.setIntakePower(0.0)
                hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
            }
        }

    fun shootBoxFirst(hw: CompBot2Hardware, shooter: ShooterImpl, flipperWait: Double = 0.3) =
        object : Group({}) {
            init {
                getScheduler()
                    .add(OneShot {
                        hw.setIntakePower(0.0)
                        hw.slider.position = SLIDER_IN
                        hw.shooterBallStop.position = SHOOTER_STOP_UP
                        hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                    })
                    .then(Wait.ms(500))
                    .then(OneShot {
                        hw.setIntakePower(1.0)
                    })
                    .then(WaitUntilContinuous(flipperWait) {
                        !hw.frontRamp.state && (hw.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
                                || hw.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)
                    })
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_UP
                    })
                    .then(Wait.ms(1000))
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_DOWN
                        hw.setIntakePower(OUTTAKE_POWER)
                    })
                    .then(Wait.ms(500))
                this.require(Locks.INTAKE_STORAGE)
            }

            override fun onFinish(completedNormally: Boolean) {
                super.onFinish(completedNormally)
//                shooter.setTarget(0.0)
                hw.setIntakePower(0.0)
                hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
            }
        }

    fun shootBoxMiddle(hw: CompBot2Hardware, shooter: ShooterImpl, flipperWait: Double = 0.3) =
        object : Group({}) {
            init {
                getScheduler()
                    .add(OneShot {
                        hw.slider.position = SLIDER_OUT
                        hw.flipper.position = FLIPPER_UP
                        hw.shooterBallStop.position = SHOOTER_STOP_UP
                        hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                    })
                    .then(Wait.ms(250))
                    .then(OneShot {
                        hw.setIntakePower(1.0)
                    })
                    .then(Wait.ms(500))
                    .then(OneShot {
                        hw.setIntakePower(0.5)
                        hw.slider.position = SLIDER_IN
                        hw.flipper.position = FLIPPER_MID
                    })
                    .then(Wait.ms(750))
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_UP
                    })
                    .then(Wait.ms(250))
                    .then(OneShot {
                        hw.setIntakePower(OUTTAKE_POWER)
                        hw.flipper.position = FLIPPER_DOWN
                    })
                    .then(Wait.ms(250))
                    .then(OneShot {
                        hw.setIntakePower(1.0)
                    })
                    .then(Wait.ms(750))
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_UP
                    })
                    //                .then(WaitUntilContinuous(flipperWait) {
//                    !hw.frontRamp.state && (hw.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
//                            || hw.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)
//                })
//                .then(OneShot {
//                    hw.flipper.position = FLIPPER_UP
//                })
                    .then(Wait.ms(1000))
                    .then(OneShot {
                        hw.setIntakePower(OUTTAKE_POWER)
                        hw.flipper.position = FLIPPER_DOWN
                    })
                    .then(Wait.ms(500))
                this.require(Locks.INTAKE_STORAGE)
            }

            override fun onFinish(completedNormally: Boolean) {
                super.onFinish(completedNormally)
//                shooter.setTarget(0.0)
                hw.setIntakePower(0.0)
                hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
            }
        }

    @JvmOverloads
    fun shoot(hw: CompBot2Hardware, shooter: ShooterImpl, flipperWait: Double = 0.15) =
        object : Group({}) {
            init {
                getScheduler()
                    .add(OneShot {
                        hw.setIntakePower(1.0)
                        hw.bottomBallStop.position = BOTTOM_STOP_STOWED
                        hw.shooterBallStop.position = SHOOTER_STOP_UP
                        hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                    })
                    .then(WaitUntilContinuous(flipperWait) {
                        !hw.frontRamp.state && (hw.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
                                || hw.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)
                    })
                    .then(OneShot {
                        hw.flipper.position = FLIPPER_UP
                    })
                    .then(Wait.ms(700))
                this.require(Locks.INTAKE_STORAGE)
            }
        }

    fun shootAfter(hw: CompBot2Hardware) = Group {
        it.add(OneShot {
            hw.flipper.position = FLIPPER_DOWN
            hw.setIntakePower(OUTTAKE_POWER)
        })
            .then(Wait.ms(500))
            .then(OneShot {
                hw.setIntakePower(0.0)
                hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
            })
    }.require(Locks.INTAKE_STORAGE)
}