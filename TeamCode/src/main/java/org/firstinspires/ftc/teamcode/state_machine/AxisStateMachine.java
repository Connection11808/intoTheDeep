package org.firstinspires.ftc.teamcode.state_machine;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sys.Lift;
import org.firstinspires.ftc.teamcode.sys.OutTakeArm;

public class AxisStateMachine extends StateMachineBase {
    public enum axis_States {

        UP(1),
        MoveOuttake(2),
        SPEC_HP(3);

        public final int value;

        private axis_States(int value)
        {
            this.value = value;
        }
    }

    private static final double TIME_DELAY_SEC = 0.3 * 1000;

    private Lift lift = null;
    private OutTakeArm outTakeArm = null;
    private ElapsedTime timer = null;

    public AxisStateMachine(Lift lift, OutTakeArm OutTakeServo)
    {
        this.lift = lift;
        this.outTakeArm = OutTakeServo;
        this.timer = new ElapsedTime();
    }

    @Override
    public void update()
    {
        if (!this.IsRunning) {
            return;
        }
        if (state == axis_States.UP.value)  {
            if (state != statePrevValue) {
                if (lift.get_lift_position() < Lift.HP_LIFT_UP_POSITION) {
                    lift.GET_FROM_HP_UP();
                }
                outTakeArm.TRANSFER();
            }

            boolean isLiftUpEnough = lift.get_lift_position() >= Lift.HP_LIFT_UP_POSITION - 100;
            boolean isReadyForNextStage = isLiftUpEnough && lift.isAxisInPosition();
            if (isReadyForNextStage) {
                setState(axis_States.MoveOuttake.value);
            }
        }
        if (state == axis_States.MoveOuttake.value) {
            if (state != statePrevValue) {
                timer.reset();
                outTakeArm.GET_SPEC_FROM_H();
            }

            if (timer.milliseconds() > TIME_DELAY_SEC) {
                setState(axis_States.SPEC_HP.value);
            }
        }

        if (state == axis_States.SPEC_HP.value) {
            lift.GET_FROM_HP_DOWN();
            outTakeArm.OUT_TAKE_GRIP_OPEN();

            if (lift.isInPosition()) {
                stop();
            }
        }

        setPrevValue();
    }
}
