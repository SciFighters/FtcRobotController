package org.firstinspires.ftc.teamcode.centerstage.util.StateMachine;

import androidx.annotation.NonNull;

public class State<OwnerClass> {
    OwnerClass owner;

    public void enter(OwnerClass owner) {
        this.owner = owner;
    }

    public void execute() {
    }

    public void exit() {
    }

    @Override
    public String toString() {
        return this.getClass().getName();
    }
}
