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
fun shootThree(shooter: Shooter, indexer: Indexer, startAt: () -> Indexer.Position = { Out1 }) = VirtualGroup {
    add(VirtualGroup {
        add(shooter.setTargetAndWait(1200.0, 0.35))
        add(indexer.goToPosition(startAt))
    })
        .then(indexer.shoot())
        .then(VirtualGroup {
            add(shooter.setTargetAndWait(1200.0, 0.35))
            add(indexer.goToPosition { next[startAt()]!! })
        }).also {
            it.inside.forEach(ITask<*>::debug)
        }
        .then(indexer.shoot())
        .then(VirtualGroup {
            add(shooter.setTargetAndWait(1200.0, 0.35))
            add(indexer.goToPosition { next[next[startAt()]]!! })
        })
        .then(indexer.shoot())
        .then(OneShot {
            shooter.setTarget(0.0)
        })
}