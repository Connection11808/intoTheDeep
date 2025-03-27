package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ServoUnitTestWithDebounce", group="Linear OpMode")
@Disabled
public class ThreeJointedIntake extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo outakeServo = null;
    private Servo outakeAxisServo = null;
    private Servo liftPitchServo = null;

    private double outake_pos = 0;
    private double outakeAxis_pos = 0;
    private double liftPitch_pos = 0;

    private final double MOVE_AMOUNT = 0.01; // Amount to move the servo each time a button is pressed

    // Variables to track button states
    private boolean lastAButtonState = false;
    private boolean lastYButtonState = false;
    private boolean lastBButtonState = false;
    private boolean lastRBButtonState = false;
    private boolean lastXButtonState = false;
    private boolean lastLBButtonState = false;
    private double outTakeGripOpen = 0.18;
    private double outTakeGripClose =  0.5;
    private double outTakePitchBasket = 0.95;
    private double PitchGetSpec = 0.0;
    private double Pitchtransfer = 0.24;
    private double PitchreadyToSpec = 0.61;
    private double PitchPointWithSpec = 0.91;
    private double outTakeAxisGetSpec =  0.28;
    private double outTakeAxisBasket =  0.4;
    private double outTakeAxisPointWithSpec =  0.9;






    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        outakeServo = hardwareMap.get(Servo.class, "outake_Servo");
        outakeAxisServo = hardwareMap.get(Servo.class, "outakeAxis_Servo");
        liftPitchServo = hardwareMap.get(Servo.class, "liftPitch_Servo");

               // Start with all servos at position 0
        outake_pos = outTakeGripOpen;
        outakeAxis_pos = outTakeAxisBasket;
        liftPitch_pos = PitchGetSpec;

        // Set the initial positions of the servos
        outakeServo.setPosition(outake_pos);
        outakeAxisServo.setPosition(outakeAxis_pos);
        liftPitchServo.setPosition(liftPitch_pos);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Handle Outake Servo movement (A and Y buttons with debounce)
            if (gamepad1.a && !lastAButtonState) {
                outake_pos += MOVE_AMOUNT;
                if (outake_pos > 1) outake_pos = 1; // Clamp to max value of 1
                outakeServo.setPosition(outake_pos);
            }
            if (gamepad1.y && !lastYButtonState) {
                outake_pos -= MOVE_AMOUNT;
                if (outake_pos < 0) outake_pos = 0; // Clamp to min value of 0
                outakeServo.setPosition(outake_pos);
            }

            // Handle Outake Axis Servo movement (B and RB buttons with debounce)
            if (gamepad1.b && !lastBButtonState) {
                outakeAxis_pos += MOVE_AMOUNT;
                if (outakeAxis_pos > 1) outakeAxis_pos = 1; // Clamp to max value of 1
                outakeAxisServo.setPosition(outakeAxis_pos);
            }
            if (gamepad1.right_bumper && !lastRBButtonState) {
                outakeAxis_pos -= MOVE_AMOUNT;
                if (outakeAxis_pos < 0) outakeAxis_pos = 0; // Clamp to min value of 0
                outakeAxisServo.setPosition(outakeAxis_pos);
            }

            // Handle Lift Pitch Servo movement (X and LB buttons with debounce)
            if (gamepad1.x && !lastXButtonState) {
                liftPitch_pos += MOVE_AMOUNT;
                if (liftPitch_pos > 1) liftPitch_pos = 1; // Clamp to max value of 1
                liftPitchServo.setPosition(liftPitch_pos);
            }
            if (gamepad1.left_bumper && !lastLBButtonState) {
                liftPitch_pos -= MOVE_AMOUNT;
                if (liftPitch_pos < 0) liftPitch_pos = 0; // Clamp to min value of 0
                liftPitchServo.setPosition(liftPitch_pos);
            }

            // Update the button states to track presses
            lastAButtonState = gamepad1.a;
            lastYButtonState = gamepad1.y;
            lastBButtonState = gamepad1.b;
            lastRBButtonState = gamepad1.right_bumper;
            lastXButtonState = gamepad1.x;
            lastLBButtonState = gamepad1.left_bumper;
            if(gamepad1.dpad_left){
                outakeServo.setPosition(outTakeGripClose);
            }


            // Display updated servo positions and runtime in telemetry
            telemetry.addData("Outake Servo Position", outake_pos);
            telemetry.addData("Outake Axis Servo Position", outakeAxis_pos);
            telemetry.addData("Lift Pitch Servo Position", liftPitch_pos);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
