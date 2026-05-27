package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitTicks
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.ACTIVE_TRACK_D
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.ACTIVE_TRACK_I
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.ACTIVE_TRACK_P
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_UP
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.Locks
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.REDUCED_TRACK_D
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.REDUCED_TRACK_I
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.REDUCED_TRACK_P
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_FAR_RANGE
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MAX_DIST
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MID_RANGE
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MIN_DIST
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.Kalman
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.systems.TurretImpl
import org.firstinspires.ftc.teamcode.systems.TurretImpl.Companion.DEADBAND_TICKS
import org.firstinspires.ftc.teamcode.systems.TurretTrack
import org.firstinspires.ftc.teamcode.systems.remover
import org.firstinspires.ftc.teamcode.systems.toDeg
import org.firstinspires.ftc.teamcode.systems.wrapAngle
import org.firstinspires.ftc.teamcode.tasks.Deferred
import org.firstinspires.ftc.teamcode.tasks.PinpointSetupTask
import org.firstinspires.ftc.teamcode.tasks.PinpointTask
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.compose
import org.firstinspires.ftc.teamcode.tasks.stopUsing
import org.firstinspires.ftc.teamcode.utilities.StaticStore
import org.firstinspires.ftc.teamcode.utilities.reportIt
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.seconds

abstract class TeleOp3(private val red: Boolean) : LinearOpMode() {
    private enum class TrackState(val htmlLabel: String) {
        Full("<font color=\"#40ff40\">ACTIVE</font>"),
        Reduced("<font color=\"#ffb040\">REDUCED</font>"),
        Off("<font color=\"#ff4040\">DISABLED</font>")
    }

    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE
    private val skew = (if (red) 1 else -1) * PI / 2

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl
    private lateinit var turret: TurretImpl
    private lateinit var turretTrack: TurretTrack

    private lateinit var kalman: Kalman

    private var intakeTask: ITask<*>? = null
    private var activeTrack: TurretTrack.TrackTask? = null
    private var activeLegacyTrack: TurretTrack.LegacyTrackTask? = null

    private var activeBind: ITask<*>? = null
    private lateinit var scheduler: FastScheduler
    private var isContinuation: Boolean = true
    private var pinpointSetupTask: PinpointSetupTask? = null

    private var kickstandUp = false

    // Offsets the Limelight camera position to the robot center,
    // accounting for the turret angle and camera mounting offset.
    // Mirrors TurretTrack.getPoseRobotFromLL() exactly so they stay consistent
    private fun offsetLLToRobotCenter(
        llX: Double, llY: Double,
        thetaTurret: Double, thetaRobot: Double
    ): Pair<Double, Double> {
        val rTurret = 6.5
        val tOffset = 0.5
        val d = sqrt(
            rTurret.pow(2.0) + tOffset.pow(2.0) -
                    2 * rTurret * tOffset * cos(PI - thetaTurret)
        )
        val x = asin(sin(PI - thetaTurret) * rTurret / d)
        val f = d * cos(x)
        val s = d * sin(x)
        val xOff = f * cos(thetaRobot) - s * sin(thetaRobot)
        val yOff = f * sin(thetaRobot) + s * cos(thetaRobot)
        return Pair(llX + xOff, llY + yOff)
    }

    private var currentTime: Long = 0
    private var prevTime: Long = 0

    private var dt: Double = 0.0

    private var prevVelX: Double = 0.0
    private var prevVelY: Double = 0.0


    private fun startTrackingFull() {
        activeTrack?.stop()
        activeLegacyTrack?.stop()
        activeBind?.stop()
        activeTrack = scheduler.add(turretTrack.track())
        currentTime = System.nanoTime()
        prevTime = currentTime
        prevVelX = hw.pinpoint.getVelX(DistanceUnit.METER)
        prevVelY = hw.pinpoint.getVelY(DistanceUnit.METER)
        TurretImpl.P = ACTIVE_TRACK_P
        TurretImpl.I = ACTIVE_TRACK_I
        TurretImpl.D = ACTIVE_TRACK_D
        activeBind = scheduler.add(compose {
            onTick {
                activeTrack ?: return@onTick true
                currentTime = System.nanoTime()
                dt = (currentTime - prevTime) * 10e9
                val velX = hw.pinpoint.getVelX(DistanceUnit.METER)
                val velY = hw.pinpoint.getVelY(DistanceUnit.METER)
                val accelX = (velX - prevVelX) / dt
                val accelY = (velY - prevVelY) / dt
                var goalPose = poseSet.goalAT
                if (kalman.stateX < -24.0) {
                    goalPose = poseSet.goalAtFAR
                }
                val hoodSpeedTurret = activeTrack!!.distance.let {
                    CompBot2Hardware.hoodAndSpeedAndTurret(
                        velX,
                        velY,
                        goalPose,
                        kalman.kalmanPose2D,
                        accelX,
                        accelY,
                        dt,
                        kalman.stateX < -24.0
                    )
                }
                shooter.setTarget(hoodSpeedTurret?.second ?: SHOOT_MID_RANGE)
                hw.hood.position = hoodSpeedTurret?.first ?: CompBot2Hardware.HOOD_50
                turret.setTarget(hoodSpeedTurret?.third ?: 0.0)
                Log.i("shooterVel", hw.getShoot1Vel().toString())

//                telemetry.addData("alphaB less than alpha", (hoodSpeedTurret.second < hoodSpeedTurret.first))
//                telemetry.addData("alphaB", hoodSpeedTurret.second)
//                telemetry.addData("alpha", hoodSpeedTurret.first)
                telemetry.addData("ppx", hw.pinpoint.getPosX(DistanceUnit.INCH))
                telemetry.addData("ppy", hw.pinpoint.getPosY(DistanceUnit.INCH))
                telemetry.addData("ppa", hw.pinpoint.getHeading(AngleUnit.RADIANS))
                telemetry.addData("statex", kalman.kalmanPose2D.x)
                telemetry.addData("statey", kalman.kalmanPose2D.y)
                telemetry.addData("statea", kalman.kalmanPose2D.a)
                telemetry.addData("counter", kalman.updateCounter)

                telemetry.addData("P", TurretImpl.P.toString())

                false
            }
        })
    }

    private fun startTrackingReduced() {
        activeTrack?.stop()
        activeLegacyTrack?.stop()
        activeBind?.stop()
        activeLegacyTrack = scheduler.add(turretTrack.trackLegacy())
        TurretImpl.P = REDUCED_TRACK_P
        TurretImpl.I = REDUCED_TRACK_I
        TurretImpl.D = REDUCED_TRACK_D
        activeBind = scheduler.add(compose {
            onTick {
                activeLegacyTrack ?: return@onTick true
                val hoodSpeed =
                    activeLegacyTrack!!.distance?.let { CompBot2Hardware.hoodAndSpeed(it) }

                if(kalman.distance > 85.0){
                    shooter.setTarget(CompBot2Hardware.SHOOT_FAR_RANGE)
                } else{
                    shooter.setTarget(hoodSpeed?.second ?: SHOOT_MID_RANGE)
                }
                hw.hood.position = hoodSpeed?.first ?: CompBot2Hardware.HOOD_50

                telemetry.addData("P", TurretImpl.P.toString())
                telemetry.update()
                false
            }
        })


    }

    private fun stopTracking() {
        activeTrack?.stop()
        activeLegacyTrack?.stop()
        activeBind?.stop()
        turret.setTarget(0.0)
        activeTrack = null
        activeLegacyTrack = null
        activeBind = null
    }

    @Suppress("IntroduceWhenSubject")
    private var trackState: TrackState
        get() = when {
            activeTrack?.getState() == ITask.State.Ticking -> TrackState.Full
            activeLegacyTrack?.getState() == ITask.State.Ticking -> TrackState.Reduced
            else -> TrackState.Off
        }
        set(value) = if (trackState != value) when (value) {
            TrackState.Full -> startTrackingFull()
            TrackState.Reduced -> startTrackingReduced()
            TrackState.Off -> stopTracking()
        } else Unit

    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hw = CompBot2Hardware(hardwareMap)

        scheduler = FastScheduler()

        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)

        val robotStartTask = scheduler.add(SentinelTask())


        if (StaticStore.duration() > 30.seconds) {
            hw.pinpoint.resetPosAndIMU()
            hw.turretEncoder.reset()
            isContinuation = false
        }

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.update()

        // Background tasks
        scheduler.add(PinpointTask(hw.pinpoint)) //come back to this
        pinpointSetupTask = scheduler.add(PinpointSetupTask(hw.pinpoint, telemetry)) //and this
        scheduler.add(compose {
            var last = System.nanoTime()
            onTick {
                val now = System.nanoTime()
                val duration = now - last
                last = now
                if (duration > 0.1e9) {
                    Log.w("timings", "took too long: last tick is %.2f ms".format(duration / 1e6))
                    Log.w("timings", reportIt(scheduler))
                }
                false
            }
        })
        val initVisual = scheduler.add(compose {
            onTick {
                initVisuals()
                false
            }
            tag(BuiltInTags.DAEMON)
        })
        shooter = robotStartTask.then(ShooterImpl(hw))
        turret = robotStartTask.then(TurretImpl(hw))
        turret.setTarget(0.0)
        turret.setPIDCoeffs(0.000_1, 0.000_2, 0.0, 220.0 * 2)
        turretTrack =
            robotStartTask.then(TurretTrack(hw.limelight, turret, hw.pinpoint, poseSet, red))
        robotStartTask.then(DriveTask())
        robotStartTask.then(OneShot {
            hw.slider.position = CompBot2Hardware.SLIDER_IN
            hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
            hw.leftKickstand.position = CompBot2Hardware.LEFT_KICKSTAND_NEUTRAL
            hw.rightKickstand.position = CompBot2Hardware.RIGHT_KICKSTAND_NEUTRAL

            pinpointSetupTask?.stop()
            initVisual.stop()
            trackState = TrackState.Full
        })
        robotStartTask.then(compose {
            onTick {
                runningVisuals()
                false
            }
            tag(BuiltInTags.DAEMON)
        })
        kalman = robotStartTask.then(
            Kalman(
                hw, hw.limelight, red, hw.pinpoint.getPosX(DistanceUnit.INCH), hw.pinpoint.getPosY(
                    DistanceUnit.INCH
                ), hw.pinpoint.getHeading(AngleUnit.RADIANS)
            )
        )

        while (opModeInInit()) {
            scheduler.tick()
        }
        robotStartTask.requestStart()
        while (opModeIsActive()) {
            scheduler.tick()
        }
    }

    fun runningVisuals() {
        telemetry.addLine(
            "<big>Live tracking is <strong>" + trackState.htmlLabel + "</strong></big>"
        )
        telemetry.addLine("<small>GP2 Back to cycle on/reduced/off</small>")
        telemetry.addLine(hw.pinpoint.position.remover.let {
            "%.2f %.2f xy %.1f deg".format(it.x, it.y, it.a.toDeg())
        })
        if (turretTrack.fault) {
            telemetry.addLine("<strong><font color=\"#ff4040\">LL fault (no new reads for at least 1 second?)</font></strong>")
        }
        activeTrack?.let {
            val (px, py) = it.getPinpointXY()
            val (lx, ly) = it.getLimelightXY()
            val (pex, pey, le) = it.getPinpointErrors()
            telemetry.addLine("XY: pin (%.2f %.2f), ll (%.2f %.2f)".format(px, py, lx, ly))
            telemetry.addLine("pin error: (%.2f %.2f), ll error: %.2f".format(pex, pey, le))
        }
//        val ekfPose = KalmanLocalization.getEstimate()
//        telemetry.addLine("EKF: %.2f %.2f xy %.1f deg".format(
//            ekfPose[0], ekfPose[1], Math.toDegrees(ekfPose[2])
//        ))
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
            if (pushValue >= Options.DRIVE_PUSH_TO_OVERRIDE && sch.getLockOwner(Locks.LOCKOUT_MOTORS) == null) {
                sch.stopUsing(Locks.DRIVE_MOTORS)
                mecanum(y, x, rx)
            }
            //reset pinpoint heading
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

            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            rotX *= 1.1 // Counteract imperfect strafing

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
        private var gp2A = false
        private var gp2B = false
        private var gp2upD = false
        private var gp2back = false

        private var gp2RT = false

        private var gp2LT = false

        fun inOut(sch: Scheduler) {
            val rb = gamepad1.right_bumper
            val lb = gamepad1.left_bumper
            val x = gamepad1.x
            val y1 = gamepad1.y
            val a2 = gamepad2.a
            val b2 = gamepad2.b
            val back2 = gamepad2.back
            val upD = gamepad2.dpad_up
            val rt2 = gamepad2.right_trigger > 0.5
            val lt2 = gamepad2.left_trigger > 0.5

            if (kickstandUp) {
                gp1RB = true
                gp1LB = true
                gp1X = true
                gp1Y = true
                gp2r = true
                gp2l = true
                gp2upD = true
                gp2A = true
                gp2B = true

            }



            if (rb && !gp1RB) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                val task = sch.add(Combo.intake(hw))
                intakeTask = task
                task.then(Combo.intakeAfter(hw))
            }
            if (a2 && !gp2A) {
                if (trackState == TrackState.Off) {
                    sch.stopUsing(Locks.INTAKE_STORAGE)
                    sch.add(VirtualGroup {
                        add(shooter.setTargetAndWait(SHOOT_MID_RANGE, 0.2))
                        add(OneShot { hw.hood.position = CompBot2Hardware.HOOD_50 })
                        add(WaitUntil { abs(turret.currentPosition()) < DEADBAND_TICKS })
                    })
                        .then(OneShot { shooter.pushThreshold = 0 })
                        .then(Combo.shoot(hw))
                        .then(OneShot { shooter.pushThreshold = shooter.defaultPushThreshold })
                        .then(Combo.shootAfter(hw))
                }
            }
            if (b2 && !gp2B) {
                val needToStopIntake = intakeTask?.getState() == ITask.State.Ticking
                sch.stopUsing(Locks.INTAKE_STORAGE)
                if (trackState != TrackState.Off) {
                    val distance = kalman.distance
                    if (distance < SHOOT_MIN_DIST) {
                        sch.add(OneShot { hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_5) })
                            .then(Wait.s(0.5))
                            .then(OneShot { hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard) })
                    } else if (distance > SHOOT_MAX_DIST) {
                        sch.add(VirtualGroup {
                            add(OneShot { hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4) })
                                .then(VirtualGroup {
                                    add(Deferred { if (needToStopIntake) Combo.intakeAfter(hw) else null })
                                    add(OneShot { shooter.pushThreshold = 0 })
                                        .then(
                                            shooter.awaitTarget(
                                                minimumDuration = 0.2,
                                                maximumDuration = 0.75
                                            )
                                        )
                                })
                                .then(Combo.shoot(hw, 0.5, intakePower = 0.6))
                                .then(OneShot {
                                    shooter.pushThreshold = shooter.defaultPushThreshold
                                })
                                .then(Combo.shootAfter(hw))
                        })
                    } else {
                        sch.add(VirtualGroup {
                            add(OneShot { hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4) })
                                .then(VirtualGroup {
                                    add(Deferred { if (needToStopIntake) Combo.intakeAfter(hw) else null })
                                    add(OneShot { shooter.pushThreshold = 0 })
                                        .then(
                                            shooter.awaitTarget(
                                                minimumDuration = 0.0,
                                                maximumDuration = 0.75
                                            )
                                        )
                                })
                                .then(Combo.shoot(hw))
                                .then(OneShot {
                                    shooter.pushThreshold = shooter.defaultPushThreshold
                                })
                                .then(Combo.shootAfter(hw))
                        })
                    }
                }
            }
            if (back2 && !gp2back) {
                trackState = when (trackState) {
                    TrackState.Full -> TrackState.Reduced
                    TrackState.Reduced -> TrackState.Off
                    TrackState.Off -> TrackState.Full
                }
            }
            if (x && !gp1X) {
                if (trackState == TrackState.Off) {
                    sch.stopUsing(Locks.INTAKE_STORAGE)
                    sch.stopUsing(Locks.DRIVE_MOTORS)
                    sch.add(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.midShoot))
                        add(shooter.setTargetAndWait(SHOOT_MID_RANGE, 0.2))
                        add(OneShot { hw.hood.position = CompBot2Hardware.HOOD_50 })
                    })
                        .then(OneShot { shooter.pushThreshold = 0 })
                        .then(Combo.shoot(hw))
                        .then(OneShot { shooter.pushThreshold = shooter.defaultPushThreshold })
                }
            }
            if (y1 && !gp1Y) {
                val needToStopIntake = intakeTask?.getState() == ITask.State.Ticking
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.stopUsing(Locks.DRIVE_MOTORS)
                sch.add(object : Group({}) {
                    var resumeAfterward = trackState

                    init {
                        getScheduler()
                            .add(Deferred { if (needToStopIntake) Combo.intakeAfter(hw) else null })
                            .then(VirtualGroup {
                                add(REmover.drive2Pose2(hw, poseSet.farShoot))
                                add(WaitTicks(1))
                                    .then(shooter.setTargetAndWait(SHOOT_FAR_RANGE, 0.5))
                                add(OneShot { hw.hood.position = CompBot2Hardware.HOOD_UP })
                                add(WaitUntil { abs(turret.currentPosition()) < DEADBAND_TICKS })
                            })
                            .then(OneShot { shooter.pushThreshold = 0 })
                            .then(Combo.shoot(hw))
                            .then(OneShot { shooter.pushThreshold = shooter.defaultPushThreshold })
                            .then(Combo.shootAfter(hw))
                        require(Locks.INTAKE_STORAGE)
                        require(Locks.DRIVE_MOTORS)
                    }

                    override fun onStart() {
                        trackState = TrackState.Off
                    }

                    override fun onFinish(completedNormally: Boolean) {
                        super.onFinish(completedNormally)
                        trackState = resumeAfterward
                    }
                })
            }
            if (lb && !gp1LB) {
                val needToStop = intakeTask?.getState() == ITask.State.Ticking
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.stopUsing(Locks.DRIVE_MOTORS)
                shooter.setTarget(0.0)
                if (needToStop) sch.add(Combo.intakeAfter(hw))
            }

            if (upD && !gp2upD && sch.getLockOwner(Locks.INTAKE_STORAGE) == null)
                shooter.setTarget(SHOOT_MID_RANGE)

            if (rt2 && !gp2RT && lt2 && !gp2LT) {
                kickstandUp = true
                sch.stopUsing(Locks.DRIVE_MOTORS)

                //TODO: Add lock so drive motors can't move while up
                sch.add(VirtualGroup {
                    add(OneShot {
                        hw.leftKickstand.position = CompBot2Hardware.LEFT_KICKSTAND_UP
                        hw.rightKickstand.position = CompBot2Hardware.RIGHT_KICKSTAND_UP
                        trackState = TrackState.Off
                        turret.setTarget(90.0)
                        shooter.setTarget(0.0)
                    })
                        .then(Wait.s(1.0))
                        .then(OneShot {
                            hw.rightKickstand.setPwmDisable()
                            hw.leftKickstand.setPwmDisable()
                            hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_7)
                        })

                    add(compose {
                        onTick {
                            false
                        }
                        require(Locks.DRIVE_MOTORS)
                        require(Locks.LOCKOUT_MOTORS)
                    })
                })

            }

            gp1RB = rb
            gp1LB = lb
            gp1X = x
            gp1Y = y1
            gp2A = a2
            gp2B = b2
            gp2upD = upD
            gp2back = back2
            gp2RT = rt2
            gp2LT = lt2
        }

        private var gp2l = false
        private var gp2r = false
        fun emerg(sch: Scheduler) {
            val lb = gamepad2.left_bumper
            val rb = gamepad2.right_bumper

            if (lb || rb) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                hw.setIntakePower(if (lb) 0.8 else -0.8)
                if (rb && !gp2r) shooter.setTarget(-500.0)
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
        return max(
            hypot(
                poseSet.shootMeasure.x - currentPos.x,
                poseSet.shootMeasure.y - currentPos.y
            ) - 5, 0.0
        )
    }

    fun lookAtGoal(): ITask<*> {
        val currentPos = hw.pinpoint.position.remover
        val phi = atan2(poseSet.shootTarget.x - currentPos.x, poseSet.shootTarget.y - currentPos.y)
        val theta1 = ((PI / 2 - phi) + PI).wrapAngle()
        return REmover.drive2Pose2(hw, REmover.RobotPose(currentPos.x, currentPos.y, theta1))
    }
}