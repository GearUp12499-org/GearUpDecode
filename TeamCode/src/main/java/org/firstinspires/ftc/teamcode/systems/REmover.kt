package org.firstinspires.ftc.teamcode.systems

import android.util.Log
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
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sign
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
        @JvmField val a: Double,
        /**
         * degrees
         */
        @JvmField val turret: Double? = null
    ) {
        @get:JvmName("asPose2D")
        val asPose2D: Pose2D get() = Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, a)
    }

    const val THRESHOLD = 0.2

    const val tipFearRatio: Double = 2.0
    const val FKP: Double = 0.1 //0.35
    const val tipFKP: Double = 0.1
    const val FKD: Double = 0.02 //0.02
    const val FKI: Double = 0.0005 // 0.0005

    //0.4, 0.07, 0.00001
    const val SKP: Double = 0.12// 0.4
    const val SKD: Double = 0.02 // 0.06
    const val SKI: Double = 0.0005 // 0.0005

    const val WKP: Double = 0.4 // 0.4

    var Wfudge: Double = 1.0

    const val WKD: Double = 0.01 //0.005
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
    fun normalize(angle: Double): Double {
        var tempAngle = angle % (2 * PI)
        if (tempAngle > PI) {
            tempAngle -= (2 * PI)
        } else if (tempAngle <= -PI) {
            tempAngle += (2 * PI)
        }
        return tempAngle
    }

    @JvmStatic
    fun angleDifference(angle1: Double, angle2: Double): Double {
        var diff = normalize(angle1) - normalize(angle2)
        diff = abs(diff)
        if(diff > PI){
            diff = (2*PI) - diff
        }

        return diff
    }

    @JvmStatic
    @JvmOverloads
    fun drive2Pose2(
        hardware: CompBot2Hardware,
        pose: RobotPose,
        maxPower: Double = 1.0,
        waypoint: Boolean = false,
        timeoutAt: Double = 1.0,
        farStrafe: Boolean = false
    ): Task<*> {
        var (tgtx, tgty, tgta) = pose

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

            var tempTargetAngle = tgta
            var sumF = 0.0
            var sumS = 0.0
            var sumW = 0.0


            override fun onStart() {
                timeout = ElapsedTime(ElapsedTime.Resolution.SECONDS)
                runtime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
                currentTime = runtime.time()
                prevTime = runtime.time()

                hardware.pinpoint.update()
                val x = hardware.pinpoint.getPosX(DistanceUnit.INCH)
                val y = hardware.pinpoint.getPosY(DistanceUnit.INCH)
                val angle = hardware.pinpoint.getHeading(AngleUnit.RADIANS)
                val deltaX = tgtx - x
                val deltaY = tgty - y

                val tempTargetAngle1 = normalize(atan2(deltaY,deltaX))
                val tempTargetAngle2 = normalize(tempTargetAngle1 + PI)
                Log.i("tempA2", tempTargetAngle2.toString())
                Log.i("tempA1", tempTargetAngle1.toString())
                tgta = normalize(tgta)

                val error1 = angleDifference(tempTargetAngle1, angle) + angleDifference(tgta, tempTargetAngle1)
                val error2 = angleDifference(tempTargetAngle2, angle) + angleDifference(tgta, tempTargetAngle2)

                if(farStrafe){
                    Wfudge = 5.0
                  if (error1 <= error2){
                      tempTargetAngle = tempTargetAngle1
                  }
                    else if (error2 < error1){
                        tempTargetAngle = tempTargetAngle2
                  }
                }
                else{
                    tempTargetAngle = tgta
                }



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
                var deltaA = tempTargetAngle - currentTheta

                deltaA %= 2 * PI
                if (deltaA > PI) {
                    deltaA -= 2 * PI
                } else if (deltaA < -PI) {
                    deltaA += 2 * PI
                }

                if (checkStop(waypoint, deltaX, deltaY, deltaA, speed, angVelocity, timeoutTime, timeoutAt)) {
                    if (timeoutTime > 1) {
                        Log.w(
                            "REMover",
                            "Timed out %s: XYA: %.4f %.4f %.4f; speed: %.4f, angvel: %.4f".format(
                                this,
                                deltaX,
                                deltaY,
                                deltaA,
                                speed,
                                angVelocity
                            )
                        )
                    }
                    else{
                        Log.w("Remover","finished")
                    }
                    if (!waypoint) {
                        hardware.frontLeft.power = 0.0
                        hardware.frontRight.power = 0.0
                        hardware.backLeft.power = 0.0
                        hardware.backRight.power = 0.0
                    }
                    return true
                }



                val f = cos(currentTheta) * deltaX + sin(currentTheta) * deltaY
                val s = sin(currentTheta) * deltaX - cos(currentTheta) * deltaY
                val w = R * deltaA

                deltaTime = max(currentTime - prevTime, 0.001)

                val vF = cos(currentTheta) * xVelocity + sin(currentTheta) * yVelocity
                val vS = sin(currentTheta) * xVelocity - cos(currentTheta) * yVelocity
                val vW = R * angVelocity

                if (abs(f) > 1.5) {
                    sumF = 0.0
                } else {
                    sumF += f * deltaTime
                }

                if (abs(s) > 1.5) {
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

//                if (abs(f) > tipFearRatio * abs(s)) {
//                    val ratio: Double = abs(s) / abs(f)
//
//                    tipFactor = (tipFKP / FKP) + (ratio * tipFearRatio) * (FKP - tipFKP / FKP)
//                }

                val tempFKP: Double = tipFactor * FKP
                val tempSKP: Double = tipFactor * SKP

                val pf: Double = tempFKP * f + FKI * sumF - FKD * vF
                val ps: Double = tempSKP * s + SKI * sumS - SKD * vS
                val pw: Double = (WKP * Wfudge) * w + WKI * sumW - WKD * vW



                val deltaAll = sqrt((f * f) + (s * s) + (w * w))

                if(farStrafe && (hypot(deltaX, deltaY) < 30.0)){
                    tempTargetAngle = tgta
                    Wfudge = 1.0
                }

                if (abs(deltaAll - prevDeltaAll) > 0.5 || currentTime < 1000) {
                    prevDeltaAll = deltaAll
                    timeout.reset()
                }
//
//

//                if ((abs(hardware.pinpoint.getVelX(DistanceUnit.INCH))>0.5) || (abs(hardware.pinpoint.getVelY(DistanceUnit.INCH))>0.5) || (abs(hardware.pinpoint.getHeadingVelocity(
//                        UnnormalizedAngleUnit.RADIANS))>0.1)) {
//                    timeout.reset()
//                    }


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

const val TWO_PI = 2 * PI

fun Double.wrapAngle(): Double {
    var actual = this
    if (actual.absoluteValue > 4 * PI)
        actual = actual.sign * actual.absoluteValue - (floor(abs(actual) / TWO_PI)) * TWO_PI
    while (actual >= PI) actual -= TWO_PI
    while (actual < -PI) actual += TWO_PI
    return actual
}

fun checkStop(
    waypoint: Boolean,
    deltaX: Double,
    deltaY: Double,
    deltaA: Double,
    speed: Double,
    angVelocity: Double,
    timeoutTime: Double,
    timeoutMax: Double
): Boolean {
    if (!waypoint) {
        return (abs(deltaX) < 0.5
                && abs(deltaY) < 0.5
                && abs(deltaA) < Math.PI / 48
                && speed < 10
                && abs(angVelocity) < Math.PI / 4
                || timeoutTime > timeoutMax)
    } else {
        return (abs(deltaX) < 6
                && abs(deltaY) < 6
                && abs(deltaA) < Math.PI / 4
                || timeoutTime > timeoutMax)
    }
}

fun Double.wrapAngleDeg(): Double {
    var actual = this
    if (actual.absoluteValue > 720.0)
        actual = actual.sign * actual.absoluteValue - (floor(abs(actual) / 360.0)) * 360.0
    while (actual >= 180.0) actual -= 360.0
    while (actual < -180.0) actual += 360.0
    return actual
}

fun Number.toDeg() = this.toDouble() * 180 / Math.PI

