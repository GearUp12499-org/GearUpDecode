package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import com.qualcomm.robotcore.robot.Robot
import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover.angleDifference
import org.firstinspires.ftc.teamcode.systems.REmover.normalize
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.toString

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

    const val FKP: Double = 0.1 //0.35
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
        if (diff > PI) {
            diff = (2 * PI) - diff
        }

        return diff
    }

    @JvmStatic
    @JvmOverloads
    fun drive2Pose2(
        hardware: CompBot2Hardware,
        pose: RobotPose,
        curveAround: RobotPose = pose,
        subSections: Int = 10,
        maxPower: Double = 1.0,
        stopCond: StopConditions = StopConditions.Default,
        timeoutAt: Double = 1.0,
        farStrafe: Boolean = false,
        rotateBack: Boolean = true
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

            var prevX = 0.0
            var prevY = 0.0

            var distanceTraveled = 0.0
            var deltaDistance = 0.0
            var fakeTgt = pose


            var tempTargetAngle = tgta
            var sumF = 0.0
            var sumS = 0.0
            var sumW = 0.0



            var estimateCurveLength = 0.0

            var SSPoses = mutableListOf<RobotPose>()

            var n = 0

            override fun onStart() {
                n = 0

                val startPos = RobotPose(hardware.pinpoint.getPosX(DistanceUnit.INCH),hardware.pinpoint.getPosY(DistanceUnit.INCH),hardware.pinpoint.getHeading(AngleUnit.RADIANS))

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

                val tempTargetAngle1 = normalize(atan2(deltaY, deltaX))
                val tempTargetAngle2 = normalize(tempTargetAngle1 + PI)
                Log.i("tempA2", tempTargetAngle2.toString())
                Log.i("tempA1", tempTargetAngle1.toString())
                tgta = normalize(tgta)

                val error1 = angleDifference(tempTargetAngle1, angle) + angleDifference(
                    tgta,
                    tempTargetAngle1
                )
                val error2 = angleDifference(tempTargetAngle2, angle) + angleDifference(
                    tgta,
                    tempTargetAngle2
                )

                if (farStrafe) {
                    Wfudge = 5.0
                    if (error1 <= error2) {
                        tempTargetAngle = tempTargetAngle1
                    } else if (error2 < error1) {
                        tempTargetAngle = tempTargetAngle2
                    }
                } else {
                    tempTargetAngle = tgta
                }




                if (curveAround != pose){
                    Log.i("StartPos", startPos.toString())
                    Log.i("EndPos", pose.toString())
//                    val startY: Double = startPos.y
//                    val curveY: Double = curveAround.y
//                    val startX: Double = startPos.x
//                    val curveX: Double = curveAround.x

                    //CHANGE
//                    estimateCurveLength = hypot(startY-curveY, startX-curveX) + hypot(curveY - tgty,curveX - tgtx)
                    var prevSS = startPos
                    for(i in 1..subSections+1 step 1){
                        Log.i("Remover", "for loop")
                        //do this to return a double not int
                        val tI = i.toDouble()
                        val tSubSections = (subSections+1).toDouble()
                        val t = (tI/tSubSections)
                        Log.i("t", t.toString())

                        val subSection = bezier(startPos,curveAround,pose,t)

                        val SSLength = hypot(subSection.x - prevSS.x, subSection.y - prevSS.y)

                        Log.i("SubSection", "length of pose # " + (i-1).toString() + " " +SSLength.toString())

                        estimateCurveLength += SSLength

                        SSPoses.add(subSection)

                        prevSS = subSection
                        Log.i("ArcLength", estimateCurveLength.toString())
                        Log.i("SubSection", "pose # " + (i-1).toString() + " " +SSPoses[i-1].toString())
                    }

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

                distanceTraveled += deltaDistance
                var bezierA = 0.0

//                if(curveAround != pose){
//                       var t = (distanceTraveled/estimateCurveLength)
//
//                        if (t>=1){
//                            t = 1.0
//                        }
//                        else if(t<=0){
//                            t = 0.001 // fudge factor cuz t = will make the robot stay at its original position
//                        }
//
//                        val bezierPose = bezier(startPos,curveAround,pose, t,)
//
//                        //get angle
//                        val tempTargetAngle1 = normalize(atan2((bezierPose.y-currentY), (bezierPose.x-currentX)))
//                        val tempTargetAngle2 = normalize(tempTargetAngle1 + PI)
//
//                        val tgta = normalize(pose.a)
//
//                        val error1 = angleDifference(tempTargetAngle1, currentTheta) + angleDifference(
//                            tgta,
//                            tempTargetAngle1
//                        )
//                        val error2 = angleDifference(tempTargetAngle2, currentTheta) + angleDifference(
//                            tgta,
//                            tempTargetAngle2
//                        )
//
//
//                        if (error1 <= error2) {
//                            bezierA= tempTargetAngle1
//                        } else if (error2 < error1) {
//                            bezierA = tempTargetAngle2
//                        }
//
//                        Log.i("BtempA2", tempTargetAngle2.toString())
//                        Log.i("BtempA1", tempTargetAngle1.toString())
//                        Log.i("BezierA", bezierA.toString())
//                            //make robot pose
//                            if(t >= 0.9){
//                                fakeTgt = pose
//                            }
////                        else if (t >= 0.75){
////                            RobotPose(bezierX, bezierY, pose.a)
////                        }
//                        else{
//                            fakeTgt = RobotPose(bezierPose.x,bezierPose.y, bezierA)
//                        }
//
//                        if(t >=1 ){
//                            Log.i("CurrentPosAtT1", currentPose.toString())
//                        }
//
//
//                }

                if(curveAround != pose){
                    if(n >= subSections){
                        n = subSections
                    }
                    val tempTargetAngle1 = normalize(atan2((SSPoses[n].y-currentY), (SSPoses[n].x-currentX)))
                    val tempTargetAngle2 = normalize(tempTargetAngle1 + PI)

                    val tgta = normalize(pose.a)

                    val error1 = angleDifference(tempTargetAngle1, currentTheta) + angleDifference(
                        tgta,
                        tempTargetAngle1
                    )
                    val error2 = angleDifference(tempTargetAngle2, currentTheta) + angleDifference(
                        tgta,
                        tempTargetAngle2
                    )


                    if (error1 <= error2) {
                        bezierA= tempTargetAngle1
                    } else if (error2 < error1) {
                        bezierA = tempTargetAngle2
                    }

                    if(n >= subSections){
                        fakeTgt = pose
                    } else if (n >= subSections - 2){
                        fakeTgt = RobotPose(SSPoses[n].x,SSPoses[n].y, pose.a)
                    } else{
                        fakeTgt = RobotPose(SSPoses[n].x, SSPoses[n].y,bezierA)
                    }
                }

                Log.i("currentTarget", fakeTgt.toString())
                Log.i("Removern", n.toString())

                val tempDeltaX = fakeTgt.x - currentX
                val tempDeltaY = fakeTgt.y - currentY
                var tempDeltaA = fakeTgt.a - currentTheta

                val deltaX = tgtx - currentX
                val deltaY = tgty - currentY
                var deltaA = tempTargetAngle - currentTheta

                deltaA %= 2 * PI
                if (deltaA > PI) {
                    deltaA -= 2 * PI
                } else if (deltaA < -PI) {
                    deltaA += 2 * PI
                }
                tempDeltaA %= 2 * PI
                if (tempDeltaA > PI) {
                    tempDeltaA -= 2 * PI
                } else if (tempDeltaA < -PI) {
                    tempDeltaA += 2 * PI
                }

                if (stopCond.evaluate.check(
                        deltaX,
                        deltaY,
                        deltaA,
                        speed,
                        angVelocity
                    ) || timeoutTime > timeoutAt
                ) {
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
                    } else {
                        Log.w("Remover", "finished")
                    }
                    if (stopCond.stopAtEnd) {
                        Log.i("Remover", "runtime " + currentTime.toString())
                        hardware.frontLeft.power = 0.0
                        hardware.frontRight.power = 0.0
                        hardware.backLeft.power = 0.0
                        hardware.backRight.power = 0.0
                    }
                    return true
                }


                val f = cos(currentTheta) * tempDeltaX + sin(currentTheta) * tempDeltaY
                val s = sin(currentTheta) * tempDeltaX - cos(currentTheta) * tempDeltaY
                val w = R * tempDeltaA

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

                val tipFactor: Double = 1.0

//                if (abs(f) > tipFearRatio * abs(s)) {
//                    val ratio: Double = abs(s) / abs(f)
//
//                    tipFactor = (tipFKP / FKP) + (ratio * tipFearRatio) * (FKP - tipFKP / FKP)
//                }

                var tempFKP: Double = tipFactor * FKP
                var tempSKP: Double = tipFactor * SKP


                //ratio the KP up so that P isn't messed up by having "endpoints" super close the actual position
                val ratio = hypot(deltaX, deltaY) / hypot(tempDeltaX, tempDeltaY)
                tempFKP *= ratio
                tempSKP *= ratio

                val pf: Double = tempFKP * f + FKI * sumF - FKD * vF
                val ps: Double = tempSKP * s + SKI * sumS - SKD * vS
                val pw: Double = (WKP * Wfudge) * w + WKI * sumW - WKD * vW


                val deltaAll = sqrt((f * f) + (s * s) + (w * w))

                if (farStrafe && (hypot(deltaX, deltaY) < 30.0)) {
                    if (rotateBack) {
                        tempTargetAngle = tgta
                    }
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

                Log.i("power", greatestPower.toString())



                hardware.frontLeft.power = pfl
                hardware.backLeft.power = pbl
                hardware.frontRight.power = pfr
                hardware.backRight.power = pbr
                prevTime = currentTime

                deltaDistance = hypot(currentX - prevX, currentY - prevY)
                prevX = currentX
                prevY = currentY

                if(abs(tempDeltaX) < 4 && abs(tempDeltaY) < 4 && curveAround != pose){
                    n++
                }

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

internal interface StopCondition {
    fun check(
        deltaX: Double,
        deltaY: Double,
        deltaA: Double,
        speed: Double,
        angVelocity: Double
    ): Boolean
}

enum class StopConditions(internal val evaluate: StopCondition, val stopAtEnd: Boolean) {
    Default(
        evaluate = object : StopCondition {
            override fun check(
                deltaX: Double,
                deltaY: Double,
                deltaA: Double,
                speed: Double,
                angVelocity: Double
            ) = (abs(deltaX) < 0.5
                    && abs(deltaY) < 0.5
                    && abs(deltaA) < Math.PI / 48
                    && speed < 10
                    && abs(angVelocity) < Math.PI / 4)
        },
        stopAtEnd = true
    ),
    Waypoint(
        evaluate = object : StopCondition {
            override fun check(
                deltaX: Double,
                deltaY: Double,
                deltaA: Double,
                speed: Double,
                angVelocity: Double
            ) = (abs(deltaX) < 6
                    && abs(deltaY) < 6
                    && abs(deltaA) < Math.PI / 4)
        },
        stopAtEnd = false
    ),
    Precision(
        evaluate = object : StopCondition {
            override fun check(
                deltaX: Double,
                deltaY: Double,
                deltaA: Double,
                speed: Double,
                angVelocity: Double
            ) = (abs(deltaX) < 0.25
                    && abs(deltaY) < 0.25
                    && abs(deltaA) < Math.PI / 96
                    && speed < 5
                    && abs(angVelocity) < Math.PI / 8)
        },
        stopAtEnd = true
    ),
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

fun lerp(p1: Double,
         p2: Double,
         t: Double): Double{
        return (1-t)*p1 + t*p2
}

fun bezier(startPos: REmover.RobotPose,
           curveAround: REmover.RobotPose,
           pose: REmover.RobotPose,
           t: Double,
           ): REmover.RobotPose{
    //get bezier x
    val x1 = lerp(startPos.x,curveAround.x, t)
    val x2 = lerp(curveAround.x, pose.x, t)
    val bezierX = lerp(x1,x2,t)

    //get bezier y
    val y1 = lerp(startPos.y,curveAround.y, t)
    val y2 = lerp(curveAround.y, pose.y, t)
    val bezierY = lerp(y1,y2,t)

    return REmover.RobotPose(bezierX, bezierY, 0.0)

}



