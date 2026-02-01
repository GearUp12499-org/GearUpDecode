package org.firstinspires.ftc.teamcode

const val VEL_LIM = 1e-4

fun problem(text: String, ok: Boolean) =
    if (ok) text
    else "<strong><font color=\"#ff4040\">$text</font></strong>"