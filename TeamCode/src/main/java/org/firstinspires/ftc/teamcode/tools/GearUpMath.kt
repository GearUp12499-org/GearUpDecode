@file:JvmName("GearUpMath")

package org.firstinspires.ftc.teamcode.tools

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.systems.REmover
import kotlin.math.PI

fun Double.wrapAngle() = when {
    this > PI -> this - 2 * PI
    this < -PI -> this + 2 * PI
    else -> this
}

val Pose2D.remover: REmover.RobotPose
    get() = REmover.RobotPose(
        this.getX(DistanceUnit.INCH),
        this.getY(DistanceUnit.INCH),
        this.getHeading(AngleUnit.RADIANS)
    )