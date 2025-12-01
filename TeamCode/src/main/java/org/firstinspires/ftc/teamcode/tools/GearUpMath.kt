@file:JvmName("GearUpMath")

package org.firstinspires.ftc.teamcode.tools

import kotlin.math.PI

fun Double.wrapAngle() = when {
    this > PI -> this - 2 * PI
    this < -PI -> this + 2 * PI
    else -> this
}