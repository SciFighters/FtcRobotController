package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class GoToState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        this.owner.setManualMode(false, 0.5);
    }

    @Override
    public void execute() {
        if (owner.areMotorsBusy()) {
            owner.stateMachine.changeState(owner.holdState);
        }
    }

    @Override
    public void exit() {

    }
}
