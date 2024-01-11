package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

public class HoldState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
    }

    @Override
    public void execute() {
        // Continue running to position with the hold power
        if (!MathUtil.approximately(owner.getPos(), owner.getTargetPos(), 10))
            owner.setMotorsPower(0.3);
    }

    @Override
    public void exit() {
        // Stop the motors when exiting the HoldState
        owner.setMotorsPower(0);
    }
}
