package org.firstinspires.ftc.teamcode.sys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class OutTakeArm {
    private Servo out_take_Servo = null;
    private Servo out_take_Axis_Servo = null;
    private Servo lift_Pitch_Servo = null;


    private double outake_pos = 0;
    private double outakeAxis_pos = 0;
    private double liftPitch_pos = 0;

    private static final double LIFT_START_POS = 0;
    private static final double OUT_TAKE_SERVO_POS = 0;
    private static final double OUT_TAKE_AXIS_SERVO_POS = 0;
    private static final double LIFT_PITCH_SERVO = 0;

    private static final double OUT_TAKE_GRIP_POS_OPEN = 0.47;
    private static final double OUT_TAKE_GRIP_POS_CLOSE = 0.58;
//    private static final double OUT_TAKE_PITCH_BASKET = 0.09;
//    private static final double PITCH_GET_SPEC = 0.83;
    private static final double OUT_TAKE_PITCH_BASKET = 0.162;
    private static final double PITCH_GET_SPEC = 0.008;

    private static final double PITCHREADY_TO_SPEC = 0.39;
//    private static final double PITCH_POINT_WITH_SPEC = 0.330;
    private static final double PITCH_POINT_WITH_SPEC = 0.117;
//    private static final double PITCHTRANSFER = 0.79;
//    private static final double PITCHTRANSFER = 0.57;
//    private static final double PITCHTRANSFER_TELEOP = 0.57;
    private static final double PITCHTRANSFER = 0.977;
    private static final double PITCHTRANSFER_TELEOP = 0.0977;
    private static final double PITCHTRANSFER_TELEOP_TELEOP = 0.075;//0.615;
    private static final double OUT_TAKE_AXIS_GET_SPEC = 1;
    private static final double OUT_TAKE_AXIS_BASKET = 0.69;
    private static final double OUT_TAKE_AXIS_TRANSFER = 0.7345;
    private static final double OUT_TAKE_AXIS_TRANSFER_TELEOP = 0.85;

    private static final double OUT_TAKE_AXIS_POINT_WITH_SPEC = 1.0;



    public OutTakeArm(Servo out_take_Servo, Servo out_take_Axis_Servo, Servo lift_Pitch_Servo)
    {
        this.out_take_Servo = out_take_Servo;
        this.out_take_Axis_Servo = out_take_Axis_Servo;
        this.lift_Pitch_Servo = lift_Pitch_Servo;
    }
    public void START_POSITION()
    {
        this.outake_pos = OUT_TAKE_SERVO_POS;
        this.outakeAxis_pos = OUT_TAKE_AXIS_BASKET;
        this.liftPitch_pos = PITCH_GET_SPEC;

        this.out_take_Servo.setPosition(outake_pos);
        this.out_take_Axis_Servo.setPosition(outakeAxis_pos);
        this.lift_Pitch_Servo.setPosition(liftPitch_pos);
    }
    public void BASKET_8_POINT()
    {
        this.lift_Pitch_Servo.setPosition(OUT_TAKE_PITCH_BASKET);//outTakePitchBasket; //high basket
        this.out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_BASKET);//outTakeAxisBasket;
    }

    public void OUT_TAKE_GRIP_OPEN()
    {
        this.out_take_Servo.setPosition(OUT_TAKE_GRIP_POS_OPEN);

    }
    public void OUT_TAKE_GRIP_CLOSE()
    {
        this.out_take_Servo.setPosition(OUT_TAKE_GRIP_POS_CLOSE);
    }

    public void TRANSFER()
    {
        this.lift_Pitch_Servo.setPosition(PITCHTRANSFER);
        this.out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_TRANSFER);

    }

    public void READY_TO_SPEC()
    {
        this.lift_Pitch_Servo.setPosition(PITCHREADY_TO_SPEC);
        this.out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_POINT_WITH_SPEC);
    }

    public void GET_SPEC_FROM_H()
    {
        this.lift_Pitch_Servo.setPosition(PITCH_GET_SPEC);
        this.out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_GET_SPEC); //getspec
    }

    public void PLACE_SPEC()
    {
        this.lift_Pitch_Servo.setPosition(PITCH_POINT_WITH_SPEC);
        this.out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_POINT_WITH_SPEC);
    }

    public void PLACE_SPEC_FINISH()
    {
        lift_Pitch_Servo.setPosition(PITCH_POINT_WITH_SPEC);
        out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_POINT_WITH_SPEC);
    }
    public void TRANSFER_TELEOP()
    {
        lift_Pitch_Servo.setPosition(PITCHTRANSFER_TELEOP);
        out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_GET_SPEC);
    }
    public void TRANSFER_TELEOP_TELEOP()
    {
        lift_Pitch_Servo.setPosition(PITCHTRANSFER_TELEOP_TELEOP);
        out_take_Axis_Servo.setPosition(OUT_TAKE_AXIS_TRANSFER_TELEOP);
    }


}
