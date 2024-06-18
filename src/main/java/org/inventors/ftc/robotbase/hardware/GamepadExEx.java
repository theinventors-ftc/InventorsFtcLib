package org.inventors.ftc.robotbase.hardware;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadExEx extends GamepadEx {

    public GamepadExEx(Gamepad gp) {
        super(gp);
    }

    public void rumble() {
        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 600)
                .build();

        gamepad.runRumbleEffect(rumbleEffect);
    }

    public void rumble(double duration) {
        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, (int)duration*1000)
                .build();

        gamepad.runRumbleEffect(rumbleEffect);
    }

    public void rumble(double duration, double stress) {
        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(stress, stress, (int)duration*1000)
                .build();

        gamepad.runRumbleEffect(rumbleEffect);
    }
}



