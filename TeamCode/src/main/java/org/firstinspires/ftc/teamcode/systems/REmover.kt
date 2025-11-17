package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
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

    /**
     * Radius of turn, inches
     */
    const val R = 7.66

    const val ROTATE_FUDGE = 1.3

    @JvmStatic
    fun speed2Power(speed: Double) = when {
        abs(speed) < 0.001 -> 0.0
        speed > 0 -> THRESHOLD + (1 - THRESHOLD) * speed
        speed < 0 -> -THRESHOLD + (1 - THRESHOLD) * speed
        else -> throw IllegalArgumentException()
    }

    @JvmStatic
    fun drive2Pose(hardware: CompBotHardware, xya: RobotPose): Task<*> {
        val (tgtx, tgty, tgta) = xya

        return object : Task.Anonymous() {
            init {
                require(CompBotHardware.Locks.DRIVE_MOTORS)
            }

            lateinit var timeout: ElapsedTime
            lateinit var runtime: ElapsedTime
            var deltaTime = 0.0
            var currentTime = 0.0
            var prevTime = 0.0
            var prevDeltaAll = 0.0

            override fun onStart() {
                timeout = ElapsedTime(ElapsedTime.Resolution.SECONDS)
                runtime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
                currentTime = runtime.time()
                prevTime = runtime.time()
            }

            override fun onTick(): Boolean {
                currentTime = runtime.time()

                val timeoutTime = timeout.time()

                hardware.pinpoint.update()

                val yVelocity = hardware.pinpoint.getVelY(DistanceUnit.INCH)
                val xVelocity = hardware.pinpoint.getVelX(DistanceUnit.INCH)

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

                if (abs(deltaX) < 0.5 && abs(deltaY) < 0.5 && abs(deltaA) < Math.PI / 24 && speed < 10 || timeoutTime > 1) {
                    hardware.frontLeft.power = 0.0
                    hardware.frontRight.power = 0.0
                    hardware.backLeft.power = 0.0
                    hardware.backRight.power = 0.0
                    return true
                }

                val r = 7.66
                val f = cos(currentTheta) * deltaX + sin(currentTheta) * deltaY
                val s = sin(currentTheta) * deltaX - cos(currentTheta) * deltaY
                val w = r * deltaA * ROTATE_FUDGE
                val deltaAll = sqrt((f * f) + (s * s) + (w * w))

                if (abs(deltaAll - prevDeltaAll) > 0.5) {
                    timeout.reset()
                }

                var dfl = f + s - w
                var dbl = f - s - w
                var dfr = f - s + w
                var dbr = f + s + w


                //rescale the four speeds so the largest is +/- 1
                var scale = max(
                    max(abs(dfl), abs(dbl)),
                    max(abs(dfr), abs(dbr))
                )

                if (scale < 0.01) {
                    scale = 0.01
                }

                dfl /= scale
                dbl /= scale
                dfr /= scale
                dbr /= scale

                deltaTime = max(currentTime - prevTime, 0.001)

                val pid =
                    KP * deltaAll + KD * (deltaAll - prevDeltaAll) / deltaTime

                if (abs(pid) < 1.0) {
                    dfl *= pid
                    dbl *= pid
                    dfr *= pid
                    dbr *= pid
                }

                val pfl = speed2Power(dfl)
                val pfr = speed2Power(dfr)
                val pbl = speed2Power(dbl)
                val pbr = speed2Power(dbr)

                hardware.frontLeft.power = pfl
                hardware.backLeft.power = pbl
                hardware.frontRight.power = pfr
                hardware.backRight.power = pbr

                return false
            }
        }
    }

    init {
        systemPackages.add(REmover::class.qualifiedName!!)
    }
}