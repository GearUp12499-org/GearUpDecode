package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "!", name = "TeleOp - Blue")
class TeleOp2Blue : TeleOp2() {
    override val negateIfRed = +1
    override val negateIfBlue = -1
}
