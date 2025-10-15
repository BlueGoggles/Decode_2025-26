package org.firstinspires.ftc.teamcode.decode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectoryActions {

    Pose2d pose;
    MecanumDrive drive;

    public TrajectoryActions(MecanumDrive drive, Pose2d pose){
        this.drive = drive;
        this.pose = pose;
    }



     public Action getSampleTrajectory(){

         TrajectoryActionBuilder sample1 = drive.actionBuilder(pose)
                 .setTangent(90)
                 .lineToX(5)
                 .lineToY(10);
//                 .turn(Math.toRadians(90));


        return sample1.build();
     }
}
