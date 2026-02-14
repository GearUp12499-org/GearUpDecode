package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.Task
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

class PinpointSetupTask(
    val pinpoint: GoBildaPinpoint2Driver,
    val telemetry: Telemetry,
    val timeTrackDuration: Duration = 1.seconds
) : Task<PinpointSetupTask>() {

    private val data = mutableListOf<Pair<TimeSource.Monotonic.ValueTimeMark, Pose2D>>()
    var velocity = 0.0; private set
    var angularVelocity = 0.0; private set

    override fun onTick(): Boolean {
        val now = TimeSource.Monotonic.markNow()
        val expiredBefore = now - timeTrackDuration

        // Expire data
        val it = data.iterator()
        for ((time, _) in it) if (time <= expiredBefore) it.remove() else break

        // Capture data
        data.add(now to pinpoint.position)

        val collectedDuration = now - data.first().first
        var totalDistance = 0.0
        var totalAngle = 0.0

        // Compute integrals
        for ((before, after) in data.zipWithNext()) {
            val poseBefore = before.second
            val poseAfter = after.second
            totalDistance += hypot(
                poseAfter.getX(DistanceUnit.INCH) - poseBefore.getX(DistanceUnit.INCH),
                poseAfter.getY(DistanceUnit.INCH) - poseBefore.getY(DistanceUnit.INCH)
            )
            totalAngle += abs(
                poseAfter.getHeading(AngleUnit.RADIANS) - poseBefore.getHeading(AngleUnit.RADIANS)
            )
        }

        velocity = totalDistance / collectedDuration.toDouble(DurationUnit.SECONDS)
        angularVelocity = totalAngle / collectedDuration.toDouble(DurationUnit.SECONDS)
        return false
    }
}