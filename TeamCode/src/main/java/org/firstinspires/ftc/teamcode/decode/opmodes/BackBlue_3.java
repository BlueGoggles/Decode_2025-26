package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;
import org.firstinspires.ftc.teamcode.decode.helper.Utility;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerStopperServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.TrajectoryActions;

@Config
@Autonomous(name = "Back Blue - Position 3", group = "BackBlue_DecodeAutonomous")
public class BackBlue_3 extends LinearOpMode {

    @Override
    public synchronized void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltMotor intakeBeltMotor = new IntakeBeltMotor(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);
        OutputAngleServo outputAngleServo = new OutputAngleServo(hardwareMap);
        KickerStopperServo kickerStopperServo = new KickerStopperServo(hardwareMap);

        TrajectoryActions trajectoryActions = new TrajectoryActions();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_2_AUTON),
                        shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, 0, 0, 180, 7, -26, -47, true)
                )
        );

        Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON, Constants.RED_LAUNCH_LOCATION_2_AUTON);

        if (Constants.PICK_BALLS_LINE_2_FLAG) {

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 11, -67, 185, false)
                    )
            );
            Utility.autonIntakeForBack(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.LINE_2_INTAKE_WAIT_TIME);

            if (Constants.RELEASE_FLAG) {
                Utility.releaseForBack(this, drive);
            }

            Actions.runBlocking(
                    new SequentialAction(
                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_2_AUTON),
                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, 0, 0, 180, 7, -26, -47, false)
                    )
            );

            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON, Constants.RED_LAUNCH_LOCATION_2_AUTON);
        }

        if (Constants.PICK_BALLS_LINE_1_FLAG) {

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 12, -46,185, false)
                    )
            );
            Utility.autonIntakeForBack(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.LINE_1_INTAKE_WAIT_TIME);


            Actions.runBlocking(
                    new SequentialAction(
                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_2_AUTON),
                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, 0, 0, 180, 7, -26, -47, false)
                    )
            );

            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON, Constants.RED_LAUNCH_LOCATION_2_AUTON);
        }

        if (Constants.PICK_BALLS_LINE_3_FLAG) {

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 10, -87, 180, false)
                    )
            );
            Utility.autonIntakeForBack(this, shooter, intakeMotor, intakeBeltMotor, kickerServo, drive, Constants.LINE_3_INTAKE_WAIT_TIME);


            Actions.runBlocking(
                    new SequentialAction(
                            outputAngleServo.setOutputAngle(Constants.RED_LAUNCH_LOCATION_2_AUTON),
                            shooter.startShooter(Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActions.getTrajectory_1_1(drive, 0, 0, 180, 7, -26, -47, false)
                    )
            );

            Utility.shoot(this, outputAngleServo, shooter, intakeMotor, intakeBeltMotor, kickerServo, Constants.DEFAULT_SHOOTER_VELOCITY_POSITION_2_AUTON, Constants.RED_LAUNCH_LOCATION_2_AUTON);
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActions.getTrajectory_1_1(drive, -1, -1, -1, 10, -50,162.5, false)
                )
        );
    }
}