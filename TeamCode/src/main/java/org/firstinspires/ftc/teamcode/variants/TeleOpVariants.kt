@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeleOp2


@TeleOp(group = "!", name = "TeleOp - Blue")
class TeleOp2Blue : TeleOp2(false)

@TeleOp(group = "!", name = "TeleOp - Red")
class TeleOp2Red : TeleOp2(true)