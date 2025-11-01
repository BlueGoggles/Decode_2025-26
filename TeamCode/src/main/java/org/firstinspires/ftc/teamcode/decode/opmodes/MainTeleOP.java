package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter2;

@TeleOp(name = "Main Teleop", group = "robot")
public class MainTeleOP extends LinearOpMode {


    @Override
    public void runOpMode() {
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

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltServo intakeBeltServo = new IntakeBeltServo(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Shooter2 shooter2 = new Shooter2(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);

        // By default we don't want to allow the lead screw to run until it has been released.
        boolean allowLeadScrew = false;
        // By default we don't want to allow the drone to be launched until the timer has elapsed.
        boolean allowDroneLauncher = false;

        // Create the timer to lock out the drone launcher.
        //ElapsedTime droneLauncherWaitTimer = new ElapsedTime();


//        robot.initializeDroneLauncher();
        sleep(400);

        double shoulder_nudge = 0;

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
        KD = 0.003;
        Kp = 0.024;
        Target_Angle = 0;
        Z__Max = 0.75;
        enableManualOverride = false;
        teleOpSpeed = 0.0;
        drive.lazyImu.get().resetYaw();

        waitForStart();

        if (opModeIsActive()) {
            // Reset the timer to start after "Start" is pressed.
            //droneLauncherWaitTimer.reset();

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
                Orientation2 = drive.lazyImu.get().getRobotYawPitchRollAngles();
                Theta_Actual = Double.parseDouble(JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
                Theta_Velocity = drive.lazyImu.get().getRobotAngularVelocity(AngleUnit.DEGREES);
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
                if (gamepad1.y) {
                    Target_Angle = -45;
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

                if (enableManualOverride) {
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


                waitForStart();

                while (opModeIsActive()) {

                    if (gamepad1.a) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        intakeMotor.startIntake(),
                                        intakeBeltServo.startIntakeBeltServo()
                                )
                        );
                    }

                    if (gamepad1.b) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        intakeMotor.stopIntake(),
                                        intakeBeltServo.stopIntakeBeltServo()

                                )
                        );
                    }

                    if (gamepad1.x) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        shooter.startShooter(1.0)
                                )

                        );
                    }

                    if (gamepad1.y) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        shooter.stopShooter()
                                )
                        );
                    }

                    if (gamepad1.start) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        shooter2.startShooter2(),
                                        kickerServo.startKickerServo()
                                )

                        );
                    }

                    if (gamepad1.back) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        shooter2.stopShooter2(),
                                        kickerServo.stopKickerServo()
                                )
                        );
                    }

                    if (gamepad1.left_bumper) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        outputAngleServo.setOutputAngle(Constants.BLUE_LAUNCH_LOCATION_1),
                                        shooter.startShooter(0.5)

                                )
                        );
                    }

                    if (gamepad1.right_bumper) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        outputAngleServo.setOutputAngle(Constants.BLUE_LAUNCH_LOCATION_2),
                                        shooter.startShooter(0.5)
                                )
                        );
                    }

                    if (gamepad1.left_trigger > 0.5) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        outputAngleServo.setOutputAngle(Constants.BLUE_LAUNCH_LOCATION_3),
                                        shooter.startShooter(0.5)
                                )
                        );
                    }


                }


            }
        }
    }
}
