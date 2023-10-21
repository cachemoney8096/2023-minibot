package frc.robot.utils;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class PoseBuffer {
    Deque<Pose2d> poseBuffer = new ArrayDeque<>();
    Deque<Double> timestampBuffer = new ArrayDeque<>();
    /**Update the buffer */
    public void update(Pose2d pose, double timestamp){
        poseBuffer.addFirst(pose);
        timestampBuffer.addFirst(timestamp);
        if(poseBuffer.size() > 50){
            poseBuffer.removeLast();
        }
        if(timestampBuffer.size() > 50){
            timestampBuffer.removeLast();
        }
    }

    public Pose2d get(double timestamp){
        List<Double> timestampList = new ArrayList<>(timestampBuffer);
        int loc = timestampList.indexOf(timestamp);
        List<Pose2d> poseList = new ArrayList<>(poseBuffer);
        return poseList.get(loc);
    }
    /*Rotation2d r = new Rotation2d(4, 9);
    Pose2d p = new Pose2d(8, 7, r);*/
}
