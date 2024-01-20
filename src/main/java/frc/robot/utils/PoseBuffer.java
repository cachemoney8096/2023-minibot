package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class PoseBuffer {
    RingBuffer<Pair<Double, Pose2d>> buffer = new RingBuffer<Pair<Double, Pose2d>>();
    public PoseBuffer(){}

    public void pushToBuffer(Pose2d pose){
        Pair<Double, Pose2d> pos = new Pair<Double, Pose2d>(Timer.getFPGATimestamp(), pose);
        buffer.addFirst(pos);
        if(buffer.size() > 50){
            buffer.removeLast();
        }
    }

    public int size(){
        return buffer.size();
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestamp){
        if(buffer.size() < 2){
            return Optional.empty();
        }
        if(buffer.getFromFirst(0).getFirst() >= timestamp && buffer.getFromLast(0).getFirst() <= timestamp){
            for(int i = 0; i < buffer.size(); i++){
                if(buffer.getFromFirst(i).getFirst() >= timestamp && buffer.getFromFirst(i+1).getFirst() <= timestamp){
                    Pair<Double, Pose2d> timedPoseA = buffer.getFromFirst(i);
                    Pair<Double, Pose2d> timedPoseB = buffer.getFromFirst(i+1);
                    double percentage = (timedPoseA.getFirst()-timestamp)/(timedPoseA.getFirst()-timedPoseB.getFirst());
                    return Optional.of(timedPoseA.getSecond().interpolate(timedPoseB.getSecond(), percentage));
                }
            }
        }
        return Optional.empty();
    }
}
