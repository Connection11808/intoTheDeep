package org.firstinspires.ftc.teamcode.OPPmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TEST_DRIVE_IDO", group = "drive")
@Disabled
public class POV_DRIVE_V2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightFrontDrive = null;
    private Motor rightBackDrive = null;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;
    private boolean SPEED_STOP = false;

    private int SPEED_C = 1; // NOTE: one because we are dividing the power by the SPEED_C
    private int SPEED_COUNT = 0;
    private boolean toggle = false;

    private IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;


    double initYaw;
    double adjustedYaw;
    @Override
    public void runOpMode() throws InterruptedException {


//        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive = new Motor(hardwareMap, "left_front");
        leftBackDrive = new Motor(hardwareMap, "left_back");
        rightFrontDrive = new Motor(hardwareMap, "right_front");
        rightBackDrive = new Motor(hardwareMap, "right_back");


        leftFrontDrive.setInverted(true);
        leftBackDrive.setInverted(true);
        rightFrontDrive.setInverted(false);
        rightBackDrive.setInverted(false);

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));

        MecanumDrive drive = new MecanumDrive(
                false,
                leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive
        );
        imu.resetYaw();



        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper){
                if (!SPEED_STOP){
                    SPEED_COUNT = SPEED_COUNT + 1;
                    SPEED_STOP = true;
                }
            }
            else SPEED_STOP = false;
            if (SPEED_COUNT == 0){
                SPEED_C = 1;
            }
            if (SPEED_COUNT == 1){
                SPEED_C = 1 / 2;
            }
            if (SPEED_COUNT == 2){
                SPEED_C = 1 / 3;
            }
            if (SPEED_COUNT == 3){
                SPEED_C = 1 / 4;
            }
            if (SPEED_COUNT > 3){
                SPEED_COUNT = 0;
            }

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            drive.driveFieldCentric(
            lateral * SPEED_C, axial * SPEED_C, yaw * SPEED_C, imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("SPEED at the moment", SPEED_C);
            telemetry.addData("SPEED count", SPEED_COUNT);
            telemetry.update();
        }
    }
}