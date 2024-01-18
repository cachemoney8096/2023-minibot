import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.PoseBuffer;

public class PoseBufferTest {
    @Test
    void test(){
        PoseBuffer poseBuffer = new PoseBuffer();
        ArrayList<Double> ts = new ArrayList<Double>();
        for(int i = 0; i < 60; i++){
            poseBuffer.pushToBuffer(new Pose2d(Math.random()*100, Math.random()*100, new Rotation2d(Math.random())));
            ts.add(Timer.getFPGATimestamp());
        }
        for(int i = 0; i < 50; i++){
            //System.out.println(poseBuffer.getPoseAtTimestamp(endTimestamp-Math.random()*(endTimestamp-startTimestamp)));
            System.out.print(ts.get(i+9)+Math.random()*10000);
            System.out.print(" - ");
            System.out.println(poseBuffer.getPoseAtTimestamp(ts.get(i+10)));
        }
    }
}
