package org.firstinspires.ftc.teamcode.perform

import android.os.Environment
import java.io.File

object PerformanceHelpers {
    const val MAX_SIZE = 1_000_000 // 1 MB
    const val MAX_4 = MAX_SIZE / 4
    const val MAX_8 = MAX_SIZE / 8

    fun intContainer(default: Int = -1) = IntArray(MAX_4) { default }
    fun longContainer(default: Long = -1L) = LongArray(MAX_8) { default }
    fun floatContainer(default: Float = -1.0f) = FloatArray(MAX_4) { default }
    fun doubleContainer(default: Double = -1.0) = DoubleArray(MAX_8) { default }

    fun findSuitableFile(): File {
        val dir = Environment.getExternalStorageDirectory().resolve("profiles")
        dir.mkdirs()
        var i = 0
        var f: File
        do {
            i++
            f = dir.resolve("profile$i.csv")
        } while (f.exists())
        return f
    }
}