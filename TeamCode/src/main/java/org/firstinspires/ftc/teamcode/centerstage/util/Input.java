package org.firstinspires.ftc.teamcode.centerstage.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;

public class Input {
    private static Gamepad gamepad1, gamepad2;
    private static Toggle[] toggles = null;

    public static void updateControls(Gamepad gamepad1_, Gamepad gamepad2_) {
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
    }

    private static Toggle[] getToggles() {
        if (toggles == null) {
            toggles = new Toggle[]{
                    new Toggle(KeyCode.Gamepad1A),
                    new Toggle(KeyCode.Gamepad1B),
                    new Toggle(KeyCode.Gamepad1X),
                    new Toggle(KeyCode.Gamepad1Y),
                    new Toggle(KeyCode.Gamepad1DpadUp),
                    new Toggle(KeyCode.Gamepad1DpadDown),
                    new Toggle(KeyCode.Gamepad1DpadLeft),
                    new Toggle(KeyCode.Gamepad1DpadRight),
                    new Toggle(KeyCode.Gamepad1Start),
                    new Toggle(KeyCode.Gamepad2A),
                    new Toggle(KeyCode.Gamepad2B),
                    new Toggle(KeyCode.Gamepad2X),
                    new Toggle(KeyCode.Gamepad2Y),
                    new Toggle(KeyCode.Gamepad2DpadUp),
                    new Toggle(KeyCode.Gamepad2DpadDown),
                    new Toggle(KeyCode.Gamepad2DpadLeft),
                    new Toggle(KeyCode.Gamepad2DpadRight),
                    new Toggle(KeyCode.Gamepad2Start)
            };
        }
        return toggles;
    }

    public static boolean GetKeyPressed(KeyCode key) {
        return key.get();
    }

    public static boolean GetKeyClicked(KeyCode key) {
        Toggle[] toggles = getToggles();
        for (Toggle t : toggles) {
            if (t.getMapping() == key) {
                return t.isClicked();
            }
        }
        return false;
    }

    public static boolean GetKeyReleased(KeyCode key) {
        Toggle[] toggles = getToggles();
        for (Toggle t : toggles) {
            if (t.getMapping() == key) {
                return t.isReleased();
            }
        }
        return false;
    }

    public static class KeyCode {
        private final Func<Boolean> returnFunc;

        private KeyCode(Func<Boolean> returnFunc) {
            this.returnFunc = returnFunc;
        }

        public boolean get() {
            return returnFunc.value();
        }

        // Define button constants
        public static final KeyCode Gamepad1A = new KeyCode(() -> gamepad1.a);
        public static final KeyCode Gamepad1B = new KeyCode(() -> gamepad1.b);
        public static final KeyCode Gamepad1X = new KeyCode(() -> gamepad1.x);
        public static final KeyCode Gamepad1Y = new KeyCode(() -> gamepad1.y);
        public static final KeyCode Gamepad1DpadUp = new KeyCode(() -> gamepad1.dpad_up);
        public static final KeyCode Gamepad1DpadDown = new KeyCode(() -> gamepad1.dpad_down);
        public static final KeyCode Gamepad1DpadLeft = new KeyCode(() -> gamepad1.dpad_left);
        public static final KeyCode Gamepad1DpadRight = new KeyCode(() -> gamepad1.dpad_right);
        public static final KeyCode Gamepad1Start = new KeyCode(() -> gamepad1.start);

        public static final KeyCode Gamepad2A = new KeyCode(() -> gamepad2.a);
        public static final KeyCode Gamepad2B = new KeyCode(() -> gamepad2.b);
        public static final KeyCode Gamepad2X = new KeyCode(() -> gamepad2.x);
        public static final KeyCode Gamepad2Y = new KeyCode(() -> gamepad2.y);
        public static final KeyCode Gamepad2DpadUp = new KeyCode(() -> gamepad2.dpad_up);
        public static final KeyCode Gamepad2DpadDown = new KeyCode(() -> gamepad2.dpad_down);
        public static final KeyCode Gamepad2DpadLeft = new KeyCode(() -> gamepad2.dpad_left);
        public static final KeyCode Gamepad2DpadRight = new KeyCode(() -> gamepad2.dpad_right);
        public static final KeyCode Gamepad2Start = new KeyCode(() -> gamepad2.start);
    }
}
