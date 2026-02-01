package org.firstinspires.ftc.teamcode

const val VEL_LIM = 1e-4

fun formatPinpointVelocity(value: Double) = when {
    value < VEL_LIM -> "%.6f".format(value)
    else -> "<strong><font color=\"#ff4040\">%.6f</font></strong>".format(value)
}

fun problem(text: String, ok: Boolean) =
    if (ok) text
    else "<strong><font color=\"#ff4040\">$text</font></strong>"