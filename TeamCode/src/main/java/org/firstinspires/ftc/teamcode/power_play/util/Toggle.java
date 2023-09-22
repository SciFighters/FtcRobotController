package org.firstinspires.ftc.teamcode.power_play.util;

public class Toggle {

    private boolean lastInput = false, pressed = false, changed = false, state = false;

    public Toggle( ) { }
    public Toggle(boolean initialState ) { this.state = initialState; }

    public void update(boolean input) {
        if (lastInput != input) { // if the input has changed,
            changed = true; // (it has changed)
            pressed = input;
            if (pressed) state = !state; // The toggle changes only when it's pressed and not when it's released
            lastInput = input;
        } else {
            changed = false; // otherwise it didn't
        }
    }

    public void set(boolean input) {
        changed = lastInput != input; // simplified
        state = input;
        lastInput = input;
    }

    public boolean toggle() {
        state = !state;
        changed = true;
        return state;
    }

    public boolean isPressed() {
        return pressed;
    }

    public boolean isChanged() {
        return changed;
    }

    public boolean getState() { return state; }

    public boolean isClicked() {
        return changed && pressed;
    }

    public boolean isReleased() { return changed && !pressed; }
    //THIS CODE IS NICE DON'T LAUGH :(
}
