package org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States;

import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class GraceHoldTimeState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        owner.handSetPower(0);
        owner.timer.reset();
    }

    @Override
    public void execute() {
//        if (owner.timer.seconds() >= owner.graceTimeLimit) {
        owner.stateMachine.changeState(owner.holdState);
//        }
    }

    @Override
    public void exit() {

    }
}
