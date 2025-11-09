package org.firstinspires.ftc.teamcode.decode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectoryActions {

     public Action getTrajectory_1_1(MecanumDrive drive, double poseX, double poseY, double poseHeading, double x, double y, double heading, boolean initialPoseFlag){

         Pose2d pose;

         if (initialPoseFlag) {
             pose = new Pose2d(poseX, poseY, Math.toRadians(poseHeading));
         } else {
             pose = drive.localizer.getPose();
         }

         TrajectoryActionBuilder builder = drive.actionBuilder(pose)
                 .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading));

         return builder.build();
     }
}
