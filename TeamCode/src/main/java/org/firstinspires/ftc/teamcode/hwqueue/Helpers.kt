package org.firstinspires.ftc.teamcode.hwqueue

import java.lang.ref.WeakReference

val <T> T.weakRef: WeakReference<T> get() = WeakReference(this)
