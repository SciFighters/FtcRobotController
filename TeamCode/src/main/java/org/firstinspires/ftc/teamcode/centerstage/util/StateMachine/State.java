package org.firstinspires.ftc.teamcode.centerstage.util.StateMachine;

public class State<OwnerClass> {
    OwnerClass owner;

    public void enter(OwnerClass owner) {
        this.owner = owner;
    }

    public void execute() {
    }

    public void exit() {
    }
}
