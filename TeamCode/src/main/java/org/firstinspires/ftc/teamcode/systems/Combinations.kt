package org.firstinspires.ftc.teamcode.systems

import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.Out1
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.Out2
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.Out3
import org.firstinspires.ftc.teamcode.tasks.debug


private val next = mapOf(
    Out1 to Out2,
    Out2 to Out3,
    Out3 to Out1
)

@JvmOverloads
fun shootThree(
    speedProvider: () -> Double,
    b: Bundle,
    startAt: (() -> Indexer.Position) = { Out1 },
    shutDownAtEnd: Boolean = true
) = VirtualGroup {
    add(VirtualGroup {
        add(b.shooter.setTargetAndWait(speedProvider, 0.35))
        add(b.indexer.goToPosition(startAt))
    })
        .then(OneShot {
            b.aprilTag.readPosition()?.let(b.hw!!::integratePositionData)
        })
        .then(b.indexer.shoot())
        .then(VirtualGroup {
            add(b.shooter.setTargetAndWait(speedProvider, 0.2, 1.0))
            add(b.indexer.goToPosition { next[startAt()]!! })
        }).also {
            it.inside.forEach(ITask<*>::debug)
        }
        .then(b.indexer.shoot())
        .then(VirtualGroup {
            add(b.shooter.setTargetAndWait(speedProvider, 0.2, 1.0))
            add(b.indexer.goToPosition { next[next[startAt()]]!! })
        })
        .then(b.indexer.shoot())
        .then(OneShot {
            if (shutDownAtEnd)
                b.shooter.setTarget(0.0)
        })
}

@JvmOverloads
fun shootThree(
    speed: Double,
    b: Bundle,
    startAt: (() -> Indexer.Position) = { Out1 },
    shutDownAtEnd: Boolean = true
) = shootThree({ speed }, b, startAt, shutDownAtEnd)