package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

@Suppress("SpellCheckingInspection")
object REmover {
    /**
     * A robot pose.
     * [x] and [y] are in inches, and [a] is in radians.
     */
    data class RobotPose(
        /**
         * inches
         */
        @JvmField val x: Double,
        /**
         * inches
         */
        @JvmField val y: Double,
        /**
         * radians
         */
        @JvmField val a: Double
    ) {
        @get:JvmName("asPose2D")
        val asPose2D: Pose2D get() = Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, a)
    }

    const val KP = 0.2
    const val KD = 43.75
    const val THRESHOLD = 0.2

    const val tipFearRatio: Double = 2.0
    const val FKP: Double = 0.35
    const val tipFKP: Double = 0.1
    const val FKD: Double = 0.01
    const val FKI: Double = 0.00001



    //0.4, 0.07, 0.00001
    const val SKP: Double = 0.4
    const val SKD: Double = 0.016
    const val SKI: Double = 0.00001

    const val WKP: Double = 0.15
    const val WKD: Double = 0.005
    const val WKI: Double = 0.0

    /**
     * Radius of turn, inches
     */
    const val R = 6.53

    const val ROTATE_FUDGE = 1.3

    @JvmStatic
    fun speed2Power(speed: Double) = when {
        abs(speed) < 0.001 -> 0.0
        speed > 0 -> THRESHOLD + (1 - THRESHOLD) * speed
        speed < 0 -> -THRESHOLD + (1 - THRESHOLD) * speed
        else -> throw IllegalArgumentException()
    }

    @JvmStatic
    @JvmOverloads
    fun drive2Pose2(
        hardware: CompBot2Hardware,
        pose: RobotPose,
        maxPower: Double = 1.0
    ): Task<*> {
        val (tgtx, tgty, tgta) = pose

        return object : Task.Anonymous() {
            init {
                require(CompBot2Hardware.Locks.DRIVE_MOTORS)
            }

            lateinit var timeout: ElapsedTime
            lateinit var runtime: ElapsedTime
            var deltaTime = 0.0
            var currentTime = 0.0
            var prevTime = 0.0
            var prevDeltaAll = 0.0

            var sumF = 0.0;
            var sumS = 0.0;
            var sumW = 0.0;

            override fun onStart() {
                timeout = ElapsedTime(ElapsedTime.Resolution.SECONDS)
                runtime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
                currentTime = runtime.time()
                prevTime = runtime.time()
            }

            override fun onTick(): Boolean {
                currentTime = runtime.time()

                val timeoutTime = timeout.time()

                val yVelocity = hardware.pinpoint.getVelY(DistanceUnit.INCH)
                val xVelocity = hardware.pinpoint.getVelX(DistanceUnit.INCH)
                val angVelocity =
                    hardware.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)

                val speed = hypot(xVelocity, yVelocity)

                val currentPose = hardware.pinpoint.position

                val currentX = currentPose.getX(DistanceUnit.INCH)
                val currentY = currentPose.getY(DistanceUnit.INCH)
                val currentTheta = currentPose.getHeading(AngleUnit.RADIANS)

                val deltaX = tgtx - currentX
                val deltaY = tgty - currentY
                var deltaA = tgta - currentTheta
                deltaA %= 2 * PI
                if (deltaA > PI) {
                    deltaA -= 2 * PI
                } else if (deltaA < -PI) {
                    deltaA += 2 * PI
                }

                if (
                    (abs(deltaX) < 0.5
                    && abs(deltaY) < 0.5
                    && abs(deltaA) < Math.PI / 48
                    && speed < 10
                    && abs(angVelocity) < Math.PI / 4)
                    || timeoutTime > 1
                ) {
                    hardware.frontLeft.power = 0.0
                    hardware.frontRight.power = 0.0
                    hardware.backLeft.power = 0.0
                    hardware.backRight.power = 0.0
                    return true
                }


                val f = cos(currentTheta) * deltaX + sin(currentTheta) * deltaY
                val s = sin(currentTheta) * deltaX - cos(currentTheta) * deltaY
                val w = R * deltaA

                deltaTime = max(currentTime - prevTime, 0.001)

                val vF = cos(currentTheta) * xVelocity + sin(currentTheta) * yVelocity;
                val vS = sin(currentTheta) * xVelocity - cos(currentTheta) * yVelocity;
                val vW = R * angVelocity

                if (abs(f) > 1) {
                    sumF = 0.0
                } else {
                    sumF += f * deltaTime
                }

                if (abs(s) > 1) {
                    sumS = 0.0
                } else {
                    sumS += s * deltaTime
                }

//                if (W < 3) {
//                    sumW = 0.0
//                } else {
//                    sumW += W * deltaTime
//                }

                var tipFactor: Double = 1.0

                if(abs(f) > tipFearRatio * abs(s)){
                    val ratio: Double = abs(s) / abs(f)

                    tipFactor = (tipFKP / FKP) + (ratio * tipFearRatio) * (FKP - tipFKP / FKP)
                }

                val tempFKP : Double = tipFactor * FKP
                val tempSKP : Double = tipFactor * SKP

                val pf: Double = tempFKP * f + FKI * sumF - FKD * vF
                val ps: Double = tempSKP * s + SKI * sumS - SKD * vS
                val pw: Double = WKP * w + WKI * sumW - WKD * vW





                val deltaAll = sqrt((f * f) + (s * s) + (w * w))

                if (abs(deltaAll - prevDeltaAll) > 0.5) {
                    timeout.reset()
                }


                var pfl = pf + ps - pw
                var pbl = pf - ps - pw
                var pfr = pf - ps + pw
                var pbr = pf + ps + pw


                //rescale the four speeds so the largest is +/- 1
                val greatestPower = max(
                    max(abs(pfl), abs(pbl)),
                    max(abs(pfr), abs(pbr))
                )

                if (greatestPower > maxPower) {
                    val scale = greatestPower / maxPower
                    pfl /= scale
                    pbl /= scale
                    pfr /= scale
                    pbr /= scale
                }

                hardware.frontLeft.power = pfl
                hardware.backLeft.power = pbl
                hardware.frontRight.power = pfr
                hardware.backRight.power = pbr

                prevDeltaAll = deltaAll
                prevTime = currentTime

                return false
            }
        }
    }


    init {
        systemPackages.add(REmover::class.qualifiedName!!)
    }
}

val Pose2D.remover: REmover.RobotPose
    get() = REmover.RobotPose(
        this.getX(DistanceUnit.INCH),
        this.getY(DistanceUnit.INCH),
        this.getHeading(AngleUnit.RADIANS)
    )

fun Double.wrapAngle() = when {
    this > PI -> this - 2 * PI
    this < -PI -> this + 2 * PI
    else -> this
}
