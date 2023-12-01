package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class HoldState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        owner.setTargetPositions(owner.getPos());
        owner.setManualMode(false, owner.getHoldPower());
    }

    @Override
    public void execute() {
        owner.setManualMode(false, owner.getHoldPower());
    }

    @Override
    public void exit() {

    }
}
