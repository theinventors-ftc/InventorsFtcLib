package org.inventors.ftc.robotbase.controllers;

import com.arcrobotics.ftclib.util.Timing;

import org.checkerframework.common.value.qual.MinLenFieldInvariant;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class StateMachine {
    private BooleanSupplier value;
    private Timing.Timer timer;
    private long triggeringTime;

    private boolean curValue, prevValue;

    private boolean justActive = false, justInactive = false;

    public StateMachine(BooleanSupplier value, long triggeringTime) {
        this.value = value;
        this.triggeringTime = triggeringTime;
        timer = new Timing.Timer(triggeringTime, TimeUnit.MILLISECONDS);
    }

    public StateMachine(BooleanSupplier value) {
        this.value = value;
        this.triggeringTime = 0;
    }

    public void update() {
        curValue = value.getAsBoolean();

        justActive = curValue && !prevValue;
        justInactive = !curValue && prevValue;

        if(justActive || justInactive) {
            timer = new Timing.Timer(triggeringTime, TimeUnit.MILLISECONDS);
            timer.start();
        }

        prevValue = curValue;
    }

    public boolean isActive() {
        return curValue;
    }

    public boolean isInactive() {
        return !curValue;
    }

    public boolean isJustActive() {
        return curValue && timer.elapsedTime() >= triggeringTime;
    }

    public boolean isJustInactive() {
        return curValue && timer.elapsedTime() >= triggeringTime;
    }
}
