@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Auto1
import org.firstinspires.ftc.teamcode.Auto2

@Autonomous
class Auto1Red : Auto1(true)

@Autonomous
class Auto1Blue : Auto1(false)

@Autonomous
class Auto2Red : Auto2(true)

@Autonomous
class Auto2Blue : Auto2(false)