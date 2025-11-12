package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;
import org.firstinspires.ftc.teamcode.decode.helper.Utility;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerStopperServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@TeleOp(name = "Main TeleOp - Blue", group = "DecodeTeleOp")
public class MainTeleOP_Blue extends LinearOpMode {

    @Override
    public synchronized void runOpMode() throws InterruptedException {
        double FL_Power;
        double FR_Power;
        double BL_Power;
        double BR_Power;
        double Gain_X;
        double Gain_Y;
        double Gain_Z;
        double Max;
        double Deadband;
        double M;
        double B;
        double Z_;
        double KD;
        double Kp;
        int Target_Angle;
        double Z__Max;
        double Joystick_X;
        double Joystick_Y;
        double Joystick_Z;
        YawPitchRollAngles Orientation2;
        double Theta_Actual;
        AngularVelocity Theta_Velocity;
        double Speed;
        double Theta_Request;
        double Theta_Command;
        double Error2;
        boolean enableManualOverride;
        double teleOpSpeed;

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltServo intakeBeltServo = new IntakeBeltServo(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);
        KickerStopperServo kickerStopperServo = new KickerStopperServo(hardwareMap);

        TrajectoryActions trajectoryActions = new TrajectoryActions();
        IMU imu = initializeIMU(drive);

        // By default we don't want to allow the lead screw to run until it has been released.
        boolean allowLeadScrew = false;
        // By default we don't want to allow the drone to be launched until the timer has elapsed.
        boolean allowDroneLauncher = false;

        // Create the timer to lock out the drone launcher.
        //ElapsedTime droneLauncherWaitTimer = new ElapsedTime();



        sleep(100);

        FL_Power = 0;
        FR_Power = 0;
        BL_Power = 0;
        BR_Power = 0;
        Gain_X = 1;
        Gain_Y = 1;
        Gain_Z = 1;
        Max = 0;
        Deadband = 0.05;
        M = 0;
        B = 0;
        Z_ = 0;
        KD = 0.002;
        Kp = 0.024;
        Target_Angle = 0;
        Z__Max = 0.75;
        enableManualOverride = true;
        teleOpSpeed = 0.0;
        imu.resetYaw();
        boolean initialPoseFlag = true;

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                Joystick_X = -1 * gamepad1.right_stick_x;
                Joystick_Y = -1 * gamepad1.right_stick_y;
                Joystick_Z = gamepad1.left_stick_x;

                M = 1 / (1 - Deadband);
                B = -Deadband / (1 - Deadband);
                if (Math.abs(Joystick_X) > Deadband) {
                    Joystick_X = (float) (M * Joystick_X + B);
                } else {
                    Joystick_X = 0;
                }
                if (Math.abs(Joystick_Y) > Deadband) {
                    Joystick_Y = (float) (M * Joystick_Y + B);
                } else {
                    Joystick_Y = 0;
                }
                if (Math.abs(Joystick_Z) > Deadband) {
                    Joystick_Z = (float) (M * Joystick_Z + B);
                } else {
                    Joystick_Z = 0;
                }
                Orientation2 = imu.getRobotYawPitchRollAngles();
                Theta_Actual = Double.parseDouble(JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
                Theta_Velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                Speed = Math.sqrt(Math.pow(Joystick_Y, 2) + Math.pow(Joystick_X, 2));
                Theta_Request = Math.atan2(Joystick_Y, Joystick_X) / Math.PI * 180;
                Theta_Command = Theta_Request - (90 - Theta_Actual);
                if (gamepad1.dpad_up) {
                    Target_Angle = 0;
                }
                if (gamepad1.dpad_right) {
                    Target_Angle = -90;
                }
                if (gamepad1.dpad_left) {
                    Target_Angle = 90;
                }
                if (gamepad1.dpad_down) {
                    if (Theta_Actual < 0) {
                        Target_Angle = -180;
                    } else {
                        Target_Angle = 180;
                    }
                }
                if (Math.abs(Target_Angle - Theta_Actual) < 180) {
                    Error2 = (int) (Target_Angle - Theta_Actual);
                } else {
                    if (Target_Angle - Theta_Actual < 0) {
                        Error2 = (int) (Target_Angle - (Theta_Actual - 360));
                    } else {
                        Error2 = (int) (Target_Angle - (Theta_Actual + 360));
                    }
                }
                Z_ = (Error2 * Kp - KD * Theta_Velocity.zRotationRate);
                if (Math.abs(Z_) > Z__Max) {
                    Z_ = (Z__Max * (Z_ / Math.abs(Z_)));
                }

                if( enableManualOverride ) {
                    // Leave Joystick_Z alone.
                } else {
                    Joystick_Z = -Z_;
                }

                Joystick_X = (Math.sin(Theta_Command / 180 * Math.PI) * Speed);
                Joystick_Y = (Math.cos(Theta_Command / 180 * Math.PI) * Speed);

                FL_Power = (-Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                FR_Power = (-Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                BL_Power = (Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                BR_Power = (Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));

                if (Math.abs(FR_Power) > Math.abs(FL_Power)) {
                    Max = Math.abs(FR_Power);
                } else {
                    Max = Math.abs(FL_Power);
                }
                if (Math.abs(BL_Power) > Max) {
                    Max = Math.abs(BL_Power);
                }
                if (Math.abs(BR_Power) > Max) {
                    Max = Math.abs(BR_Power);
                }

//                if (gamepad1.left_trigger > Constants.ZERO_POWER) {
//                    teleOpSpeed = Constants.TELEOP_MODIFIED_SPEED;
//                } else {
                teleOpSpeed = Constants.TELEOP_DEFAULT_SPEED;
//                }

                if (Max > teleOpSpeed) {
                    FR_Power = (FR_Power * teleOpSpeed) / Max;
                    FL_Power = (FL_Power * teleOpSpeed) / Max;
                    BR_Power = (BR_Power * teleOpSpeed) / Max;
                    BL_Power = (BL_Power * teleOpSpeed) / Max;
                }


                drive.leftFront.setPower(-FL_Power);
                drive.rightFront.setPower(FR_Power);
                drive.leftBack.setPower(-BL_Power);
                drive.rightBack.setPower(BR_Power);


                // This variable controls whether we are manually steering or auto steering.
                if(gamepad1.back) {
                    enableManualOverride = !enableManualOverride;
                }

                // Press this button to reset the yaw during Teleop. Only allow this to happen if we are in manual mode.
                if (gamepad1.y && enableManualOverride) {
                    imu.resetYaw();
                }

                // Setup buttons for actions below

                // GamePad 1 actions

                if (gamepad1.right_bumper) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    intakeMotor.startIntake(),
                                    intakeBeltServo.startIntakeBeltServo()
                            )
                    );
                }

                if (gamepad1.left_bumper) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    intakeMotor.stopIntake(),
                                    intakeBeltServo.stopIntakeBeltServo()

                            )
                    );
                }

                if (gamepad1.right_trigger > 0.5) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    shooter.stopShooter(),
//                                    kickerServo.stopKickerServo(),
                                    intakeMotor.reverseIntake(),
                                    intakeBeltServo.reverseIntakeBeltServo()
                            )
                    );
                }

//                if (gamepad1.b) {
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    trajectoryActions.getTrajectory_1_1(drive, 0, 0, 180, 3, 69, -52, initialPoseFlag),
//                                    outputAngleServo.setOutputAngle(Constants.BLUE_LAUNCH_LOCATION_2)
//                            )
//                    );
//                    initialPoseFlag = false;
//                }


                // GamePad 2 actions

                if (gamepad2.b) {
                    Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltServo, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2, Constants.BLUE_LAUNCH_LOCATION_2);
                }

                if (gamepad2.a) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    shooter.stopShooter(),
                                    intakeMotor.stopIntake(),
                                    intakeBeltServo.stopIntakeBeltServo()
//                                    kickerServo.stopKickerServo()
                            )
                    );
                }

                if (gamepad2.right_trigger > 0.5) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    shooter.adjustShooterSpeed(true)
                            )
                    );
                }

                if (gamepad2.left_trigger > 0.5) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    shooter.adjustShooterSpeed(false)
                            )
                    );
                }

                if (gamepad2.x) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    shooter.stopShooter()
                            )
                    );
                }

                if (gamepad2.y) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    shooter.reverseShooter(),
                                    intakeBeltServo.reverseIntakeBeltServo()
                            )
                    );
                }


                if (gamepad2.right_bumper) {
//                    Actions.runBlocking(
//                            new ParallelAction(
//                                    kickerServo.startKickerServo(0.25)
//                            )
//
//                    );
                }

                if (gamepad2.left_bumper) {
//                    Actions.runBlocking(
//                            new ParallelAction(
//                                    kickerServo.stopKickerServo()
//                            )
//                    );
                }

                if (gamepad2.start) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    outputAngleServo.adjustOutputAngle(true)
                            )
                    );
                }

                if (gamepad2.back) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    outputAngleServo.adjustOutputAngle(false)
                            )
                    );
                }


                telemetry.addData("Shooter Velocity: ", shooter.getLeftWheel().getVelocity());
                telemetry.addData("Output Angle Position: ", outputAngleServo.getOutputAngleServo().getPosition());
//                telemetry.addData("Kicker Position: ", kickerServo.getKickerServo().getPosition());
                telemetry.addData("Current Pose: ", drive.localizer.getPose());

                telemetry.update();

            }
        }
    }

    public IMU initializeIMU(MecanumDrive drive) {

        drive.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Control Hub IMU Parameters
//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        // Expansion Hub IMU Parameters
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();

        return  imu;
    }

}
