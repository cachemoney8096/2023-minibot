package frc.robot.utils;
import java.util.ArrayDeque;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
public class PoseBuffer {
    private ArrayDeque<Pair<Double, Pose2d>> aDeque = new ArrayDeque<Pair<Double, Pose2d>>();
    public PoseBuffer() {}
    /** Appends pair of timestamp and Robot Pose (x, y, rotation) to the front of the ArrayDeque, 
     * If the deque size is bigger than 50, we push out the earliest pose; the first one pushed into the deque
     * */
    public void pushToBuffer(Pose2d pose) {
        double curTime = Timer.getFPGATimestamp();
        Pair<Double, Pose2d> pair = new Pair<Double, Pose2d>(curTime, pose);
        this.aDeque.addFirst(pair);
        if (aDeque.size() > 50) aDeque.removeLast();
    }

}
