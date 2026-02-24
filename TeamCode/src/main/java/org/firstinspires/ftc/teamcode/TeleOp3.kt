package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.WaitTicks
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.Locks
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_FAR_RANGE
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MID_RANGE
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.systems.TurretImpl
import org.firstinspires.ftc.teamcode.systems.TurretTrack
import org.firstinspires.ftc.teamcode.systems.remover
import org.firstinspires.ftc.teamcode.systems.toDeg
import org.firstinspires.ftc.teamcode.systems.wrapAngle
import org.firstinspires.ftc.teamcode.tasks.PinpointSetupTask
import org.firstinspires.ftc.teamcode.tasks.PinpointTask
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.compose
import org.firstinspires.ftc.teamcode.tasks.isAliveOrQueued
import org.firstinspires.ftc.teamcode.tasks.stopUsing
import org.firstinspires.ftc.teamcode.utilities.StaticStore
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.time.Duration.Companion.seconds

abstract class TeleOp3(private val red: Boolean) : LinearOpMode() {
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE
    private val skew = (if (red) 1 else -1) * PI / 2

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl
    private lateinit var turret: TurretImpl
    private lateinit var turretTrack: TurretTrack
    private var activeTrack: TurretTrack.TrackTask? = null
    private var activeBind: ITask<*>? = null
    private lateinit var scheduler: FastScheduler
    private var isContinuation: Boolean = true
    private var pinpointSetupTask: PinpointSetupTask? = null

    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hw = CompBot2Hardware(hardwareMap)
        scheduler = FastScheduler()

        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)

        if (StaticStore.duration() > 30.seconds) {
            hw.pinpoint.resetPosAndIMU()
            isContinuation = false
        }

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.update()

        // Background tasks
        scheduler.add(PinpointTask(hw.pinpoint))
        pinpointSetupTask = scheduler.add(PinpointSetupTask(hw.pinpoint, telemetry))
        val initVisual = scheduler.add(compose {
            onTick {
                initVisuals()
                false
            }
            tag(BuiltInTags.DAEMON)
        })
        val robotStartTask = scheduler.add(SentinelTask())
        shooter = robotStartTask.then(ShooterImpl(hw))
        turret = robotStartTask.then(TurretImpl(hw))
        turret.setTarget(0.0)
        turretTrack =
            robotStartTask.then(TurretTrack(hw.limelight, turret, hw.pinpoint, poseSet, red))
        robotStartTask.then(DriveTask())
        robotStartTask.then(OneShot {
            hw.slider.position = CompBot2Hardware.SLIDER_IN
            hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED

            pinpointSetupTask?.stop()
            initVisual.stop()
//            turret.setTarget(45.0)
            startTracking()
        })
        robotStartTask.then(compose {
            onTick {
                runningVisuals()
                false
            }
            tag(BuiltInTags.DAEMON)
        })

        while (opModeInInit()) {
            scheduler.tick()
        }
        robotStartTask.requestStart()
        while (opModeIsActive()) {
            scheduler.tick()
        }
    }

    fun startTracking() {
        activeTrack?.stop()
        activeBind?.stop()
        activeTrack = scheduler.add(turretTrack.track())
        activeBind = scheduler.add(compose {
            onTick {
                activeTrack ?: return@onTick true
                val hoodSpeed =
                    activeTrack!!.distance?.let { CompBot2Hardware.hoodAndSpeed(it) }
                shooter.setTarget(hoodSpeed?.second ?: SHOOT_MID_RANGE)
                hw.hood.position = hoodSpeed?.first ?: CompBot2Hardware.HOOD_50
                false
            }
        })
    }

    fun stopTracking() {
        activeTrack?.stop()
        activeBind?.stop()
        activeTrack = null
        activeBind = null
    }

    fun runningVisuals() {
        telemetry.addLine(
            "<big>Live tracking is <strong>" +
                    (if (activeTrack?.getState() == ITask.State.Ticking) "<font color=\"#40ff40\">ACTIVE</font>"
                    else "<font color=\"#ff4040\">DISABLED</font>")
                    + "</strong></big>"
        )
        telemetry.addLine("<small>GP2 Back to enable/disable</small>")
        telemetry.addLine(hw.pinpoint.position.remover.let {
            "%.2f %.2f xy %.1f deg".format(it.x, it.y, it.a.toDeg())
        })
        if (turretTrack.fault) {
            telemetry.addLine("<strong><font color=\"#ff4040\">LL fault (no new reads for at least 1 second?)</font></strong>")
        }
        telemetry.update()
    }

    fun initVisuals() {
        if (isContinuation)
            telemetry.addLine("<big>Localization <strong><font color=\"#40ff40\">retained</font></strong></big>")
        else
            telemetry.addLine("<big>Localization <strong><font color=\"#ffb040\">reset</font></strong></big>")

        telemetry.addLine(hw.pinpoint.position.remover.let {
            "%.2f %.2f xy %.1f deg".format(it.x, it.y, it.a.toDeg())
        })

        pinpointSetupTask?.let {
            telemetry.addLine()
            telemetry.addData(
                "Linear velo (in/s)",
                problem("%.6f".format(it.velocity), it.velocity < VEL_LIM)
            )
            telemetry.addData(
                "Angular velo (rad/s)",
                problem("%.6f".format(it.angularVelocity), it.angularVelocity < VEL_LIM)
            )
        }
        telemetry.update()
    }

    private inner class DriveTask : Task<DriveTask>() {
        private inline val sch: Scheduler get() = super.scheduler!!

        override fun onTick(): Boolean {
            val sch = sch

            mecanumDispatcher(sch)
            inOut(sch)
            emerg(sch)

            return false
        }

        fun mecanumDispatcher(sch: Scheduler) {
            val y = -gamepad1.left_stick_y.toDouble()
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            if (sch.getLockOwner(Locks.DRIVE_MOTORS) == null) mecanum(y, x, rx)
            val pushValue = max(hypot(x, y), rx)
            if (pushValue >= Options.DRIVE_PUSH_TO_OVERRIDE) {
                sch.stopUsing(Locks.DRIVE_MOTORS)
                mecanum(y, x, rx)
            }

            if (gamepad1.backWasPressed()) {
                sch.stopUsing(Locks.DRIVE_MOTORS)

                val pos = hw.pinpoint.position
                val new = Pose2D(
                    DistanceUnit.INCH,
                    pos.getX(DistanceUnit.INCH),
                    pos.getY(DistanceUnit.INCH),
                    AngleUnit.RADIANS,
                    0.0
                )
                hw.pinpoint.position = new
            }
        }

        fun mecanum(y: Double, x: Double, rx: Double) {
            val botHeading: Double = hw.pinpoint.getHeading(AngleUnit.RADIANS) + skew

            // Rotate the movement direction counter to the bot's rotation
            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            rotX *= 1.1 // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower = (rotY + rotX + rx) / denominator
            val backLeftPower = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower = (rotY + rotX - rx) / denominator

            hw.frontLeft.power = frontLeftPower
            hw.backLeft.power = backLeftPower
            hw.frontRight.power = frontRightPower
            hw.backRight.power = backRightPower
        }

        private var gp1RB = false
        private var gp1LB = false
        private var gp1X = false
        private var gp1Y = false
        private var gp2Y = false
        private var gp2A = false
        private var gp2B = false
        private var gp2upD = false
        private var gp2back = false

        fun inOut(sch: Scheduler) {
            val rb = gamepad1.right_bumper
            val lb = gamepad1.left_bumper
            val x = gamepad1.x
            val y1 = gamepad1.y
            val a2 = gamepad2.a
            val b2 = gamepad2.b
            val back2 = gamepad2.back
            val upD = gamepad2.dpad_up

            if (rb && !gp1RB) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.add(Combo.intake(hw))
            }
            if (a2 && !gp2A) {
                if (!(activeTrack?.isAliveOrQueued() ?: false)) {
                    sch.stopUsing(Locks.INTAKE_STORAGE)
                    sch.add(VirtualGroup {
                        add(shooter.setTargetAndWait(SHOOT_MID_RANGE, 0.2))
                        add(OneShot {
                            hw.hood.position = CompBot2Hardware.HOOD_50
                        })
                        add(WaitUntil {
                            abs(turret.currentPosition()) < 10
                        })
                    })
                        .then(Combo.shoot(hw))
                        .then(Combo.shootAfter(hw))
                }
            }
            if (b2 && !gp2B) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                // If we're in live tracking mode
                if (activeTrack?.isAliveOrQueued() ?: false)
                    sch.add(VirtualGroup {
                        add(shooter.awaitTarget(0.2))
                            .then(Combo.shoot(hw))
                            .then(Combo.shootAfter(hw))
                    })
                // If we're... not
                else sch.add(VirtualGroup {
//                    val track = add(turretTrack.trackLegacy())
//                    val bind = add(compose {
//                        onTick {
//                            val hoodSpeed =
//                                track.distance?.let { CompBot2Hardware.hoodAndSpeed(it) }
//                            shooter.setTarget(hoodSpeed?.second ?: SHOOT_MID_RANGE)
//                            hw.hood.position = hoodSpeed?.first ?: CompBot2Hardware.HOOD_50
//                            false
//                        }
//                    })
//                    add(shooter.awaitTarget(0.2))
//                        .then(Combo.shoot(hw))
//                        .then(OneShot {
//                            track.finish()
//                            bind.finish()
//                        })
//                        .then(Combo.shootAfter(hw))
                })
            }
            if (back2 && !gp2back) {
                if (activeTrack?.isAliveOrQueued() ?: false) stopTracking()
                else startTracking()
            }
            if (x && !gp1X) {
                if (!(activeTrack?.isAliveOrQueued() ?: false)) {
                    sch.stopUsing(Locks.INTAKE_STORAGE)
                    sch.stopUsing(Locks.DRIVE_MOTORS)
                    sch.add(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.midShoot))
                        add(shooter.setTargetAndWait(SHOOT_MID_RANGE, 0.2))
                        add(OneShot {
                            hw.hood.position = CompBot2Hardware.HOOD_50
                        })
                    }).then(Combo.shoot(hw))
                }
            }
            if (y1 && !gp1Y) {
                // Temporarily suspend tracking
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.stopUsing(Locks.DRIVE_MOTORS)
                sch.add(object : Group({}) {
                    var resumeAfterward = (activeTrack?.isAliveOrQueued() ?: false)

                    init {
                        getScheduler()
                            .add(VirtualGroup {
                                add(REmover.drive2Pose2(hw, poseSet.farShoot))
                                add(WaitTicks(1))
                                    .then(shooter.setTargetAndWait(SHOOT_FAR_RANGE, 0.5))
                                add(OneShot {
                                    hw.hood.position = CompBot2Hardware.HOOD_UP
                                })
                                add(WaitUntil {
                                    abs(turret.currentPosition()) < 10
                                })
                            })
                            .then(Combo.shoot(hw))
                            .then(Combo.shootAfter(hw))
                        require(Locks.INTAKE_STORAGE)
                        require(Locks.DRIVE_MOTORS)
                    }

                    override fun onStart() {
                        stopTracking()
                    }

                    override fun onFinish(completedNormally: Boolean) {
                        super.onFinish(completedNormally)
                        if (resumeAfterward) startTracking()
                    }
                })
            }
            if (lb && !gp1LB) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.stopUsing(Locks.DRIVE_MOTORS)
                shooter.setTarget(0.0)
            }

            if (upD && !gp2upD && sch.getLockOwner(Locks.INTAKE_STORAGE) == null)
                shooter.setTarget(SHOOT_MID_RANGE)

            gp1RB = rb
            gp1LB = lb
            gp1X = x
            gp1Y = y1
            gp2A = a2
            gp2B = b2
            gp2upD = upD
            gp2back = back2
        }

        private var gp2l = false
        private var gp2r = false
        fun emerg(sch: Scheduler) {
            /*
            button prime shooter motor
            dpad to manually adjust shooter angle
            intake/outake buttons
             */
            val lb = gamepad2.left_bumper
            val rb = gamepad2.right_bumper

            if (lb || rb) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                hw.setIntakePower(if (lb) 0.8 else -0.8)
                if (rb && !gp2r) {
                    shooter.setTarget(-500.0)
                }
                if (!rb && gp2r) shooter.setTarget(0.0)
            } else if (gp2l || gp2r) {
                hw.setIntakePower(0.0)
                shooter.setTarget(0.0)
            }

            gp2l = lb
            gp2r = rb
        }
    }

    fun getDistanceToGoal(): Double {
        val currentPos = hw.pinpoint.position.remover
        val distance =
            max(
                hypot(
                    poseSet.shootMeasure.x - currentPos.x,
                    poseSet.shootMeasure.y - currentPos.y
                ) - 5, 0.0
            )
        return distance
    }

    fun lookAtGoal(): ITask<*> {
        val currentPos = hw.pinpoint.position.remover
        val phi = atan2(poseSet.shootTarget.x - currentPos.x, poseSet.shootTarget.y - currentPos.y)
        val theta1 = ((PI / 2 - phi) + PI).wrapAngle()
        return REmover.drive2Pose2(
            hw,
            REmover.RobotPose(currentPos.x, currentPos.y, theta1)
        )
    }
}