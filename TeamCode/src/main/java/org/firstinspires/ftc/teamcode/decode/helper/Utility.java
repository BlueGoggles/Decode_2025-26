package org.firstinspires.ftc.teamcode.decode.helper;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;

public class Utility {

    public static void shoot(LinearOpMode opMode, OutputAngleServo outputAngleServo, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltServo intakeBeltServo, KickerServo kickerServo, int shooterSpeed, String launchLocation) throws InterruptedException {
        Actions.runBlocking(
                new SequentialAction(
                        outputAngleServo.setOutputAngle(launchLocation),
                        shooter.startShooter(shooterSpeed)
                )
        );
//        wait(500);
//        Actions.runBlocking(
//                new SequentialAction(
//                        kickerStopperServo.kickerStopperServoOff()
//                )
//        );
        opMode.wait(500);
        Actions.runBlocking(
                new ParallelAction(
                        intakeMotor.startIntake(),
                        intakeBeltServo.startIntakeBeltServo()
//                        kickerServo.startKickerServo(kickerServo.getKickerServo().getPosition() + 0.13)
                )
        );
        opMode.wait(7000);
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

    public static void drive(MecanumDrive drive, double power) {
        drive.leftFront.setPower(0.18);
        drive.rightFront.setPower(0.18);
        drive.leftBack.setPower(0.18);
        drive.rightBack.setPower(0.18);
    }

    public static void autonIntake(Shooter shooter, IntakeMotor intakeMotor, IntakeBeltServo intakeBeltServo, KickerServo kickerServo) {
        Actions.runBlocking(
                new SequentialAction(
//                        kickerServo.stopKickerServo(),
                        shooter.stopShooter(),
                        intakeMotor.startIntake(),
                        intakeBeltServo.startIntakeBeltServo()
                )
        );
    }
}
