package org.inventors.ftc.robotbase.hardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadExEx extends GamepadEx {
    Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 600)
            .build();

    public GamepadExEx(Gamepad gp) {
        super(gp);
    }

    public GamepadExEx(Gamepad gp, Gamepad.RumbleEffect effect) {
        super(gp);
        this.rumbleEffect = effect;
    }

    public void rumble() {
        gamepad.runRumbleEffect(rumbleEffect);
    }
}
