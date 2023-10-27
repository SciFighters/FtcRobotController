package org.firstinspires.ftc.teamcode.centerstage.util;

import org.firstinspires.ftc.robotcore.external.Func;

public enum KeyCode {
    Joystick1A(Input.getAKey1),
    Joystick1B(Input.getBKey1),
    Joystick1X(Input.getXKey1),
    Joystick1Y(Input.getYKey1),
    Joystick1DpadUp(Input.getDpadUpKey1),
    Joystick1DpadDown(Input.getDpadDownKey1),
    Joystick1DpadLeft(Input.getDpadLeftKey1),
    Joystick1DpadRight(Input.getDpadRightKey1),
    Joystick1Start(Input.getStartKey1);

    private Func<Boolean> returnFunc;

    KeyCode(Func<Boolean> returnFunc) {
        this.returnFunc = returnFunc;
    }

    public Func<Boolean> get() {
        return returnFunc;
    }

    public void remap(Func<Boolean> newFunc) {
        this.returnFunc = newFunc;
    }
}
