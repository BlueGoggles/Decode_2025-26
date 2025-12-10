package org.firstinspires.ftc.teamcode.decode.helper;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;

public class Utility {

    public static void shoot(LinearOpMode opMode, OutputAngleServo outputAngleServo, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltMotor intakeBeltMotor, KickerServo kickerServo, int shooterSpeed, String launchLocation) throws InterruptedException {
//        Actions.runBlocking(
//                new SequentialAction(
//                        outputAngleServo.setOutputAngle(launchLocation),
//                        shooter.startShooter(shooterSpeed)
//                )
//        );
//        wait(500);
//        Actions.runBlocking(
//                new SequentialAction(
//                        kickerStopperServo.kickerStopperServoOff()
//                )
//        );
//        opMode.wait(500);
        Actions.runBlocking(
                new ParallelAction(
                        intakeMotor.startIntake(),
                        intakeBeltMotor.startIntakeBeltMotor()
//                        kickerServo.startKickerServo(kickerServo.getKickerServo().getPosition() + 0.13)
                )
        );
        opMode.wait(2000);
//        Actions.runBlocking(
//                new SequentialAction(
//                        kickerServo.startKickerServo(kickerServo.getKickerServo().getPosition() + 0.1)
//                )
//        );
//        opMode.wait(2000);
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        kickerServo.startKickerServo(Constants.KICKER_SERVO_HOME_POSITION)
//                )
//        );
    }

    public static void shootTeleOp(LinearOpMode opMode, OutputAngleServo outputAngleServo, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltMotor intakeBeltMotor, KickerServo kickerServo, int shooterSpeed, String launchLocation) throws InterruptedException {
        Actions.runBlocking(
                new SequentialAction(
                        outputAngleServo.setOutputAngle(launchLocation),
                        shooter.startShooter(shooterSpeed)
                )
        );

        opMode.wait(750);
        Actions.runBlocking(
                new ParallelAction(
                        intakeMotor.startIntake()
                )
        );
        intakeBeltMotor.getIntakeBeltMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBeltMotor.getIntakeBeltMotor().setPower(0.5);
    }

    public static void drive(MecanumDrive drive, double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        drive.leftFront.setPower(leftFrontPower);
        drive.rightFront.setPower(rightFrontPower);
        drive.leftBack.setPower(leftBackPower);
        drive.rightBack.setPower(rightBackPower);
    }

    public static void drive(MecanumDrive drive, double power) {
        drive(drive, power, power, power, power);
    }

    public static void autonIntake(LinearOpMode opMode, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltMotor intakeBeltMotor, KickerServo kickerServo, MecanumDrive drive, int intakeWaitTime) throws InterruptedException {
        Actions.runBlocking(
                new SequentialAction(
                        shooter.stopShooter(),
                        intakeMotor.startIntake(),
                        intakeBeltMotor.startIntakeBeltMotor()
                )
        );
        Utility.drive(drive, 1.0);

        opMode.wait(intakeWaitTime);

        Utility.drive(drive, 0);

        opMode.wait(500);

        Actions.runBlocking(
                new SequentialAction(
                        intakeBeltMotor.stopIntakeBeltMotor()
                )
        );
    }

    public static void autonIntakeForBack(LinearOpMode opMode, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltMotor intakeBeltMotor, KickerServo kickerServo, MecanumDrive drive, int intakeWaitTime) throws InterruptedException {
        Actions.runBlocking(
                new SequentialAction(
                        shooter.stopShooter(),
                        intakeMotor.startIntake(),
                        intakeBeltMotor.startIntakeBeltMotor()
                )
        );
        Utility.drive(drive, 1.0);

        opMode.wait(intakeWaitTime);

        Utility.drive(drive, 0);

        opMode.wait(500);

        Actions.runBlocking(
                new SequentialAction(
                        intakeMotor.stopIntake(),
                        intakeBeltMotor.stopIntakeBeltMotor()
                )
        );
    }

    public static void autonIntakeHumanArea(LinearOpMode opMode, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltMotor intakeBeltMotor, KickerServo kickerServo, MecanumDrive drive, int intakeWaitTime) throws InterruptedException {
        double speed = 0.5;

        Actions.runBlocking(
                new SequentialAction(
                        shooter.stopShooter(),
                        intakeMotor.startIntake(),
                        intakeBeltMotor.startIntakeBeltMotor()
                )
        );
        Utility.drive(drive, 0.5);

        opMode.wait(400);
        drive(drive, speed, -speed, -speed, speed); // Right strafe
        opMode.wait(200);
        Utility.drive(drive, speed);
        opMode.wait(400);
        drive(drive, -speed, speed, speed, -speed); // Left strafe
        opMode.wait(300);
        Utility.drive(drive, speed);
        opMode.wait(400);


        Actions.runBlocking(
                new SequentialAction(
                        intakeBeltMotor.stopIntakeBeltMotor()
                )
        );



    }

    public static void release(LinearOpMode opMode, MecanumDrive drive) throws InterruptedException {
        double speed = 0.5;

        drive(drive, -speed);
        opMode.wait(500);
//        drive(drive, -speed, speed, speed, -speed); // Left strafe
        drive(drive, speed, -speed, -speed, speed); // Right strafe
        opMode.wait(300);
        drive(drive, speed);
        opMode.wait(500);
        drive(drive, 0);
        opMode.wait(1000);
    }

    public static void releaseForBack(LinearOpMode opMode, MecanumDrive drive) throws InterruptedException {
        double speed = 0.5;

        drive(drive, -speed);
        opMode.wait(300);
//        drive(drive, -speed, speed, speed, -speed); // Left strafe
        drive(drive, speed, -speed, -speed, speed); // Right strafe
        opMode.wait(350);
        drive(drive, speed);
        opMode.wait(300);
        drive(drive, 0);
        opMode.wait(1000);
    }
}
