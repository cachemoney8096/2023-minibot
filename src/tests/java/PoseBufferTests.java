import static org.junit.Assert;
import org.junit.*;
import java.util.ArrayDeque;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer; 
public class PoseBufferTests {
    @Test
    public void poseBufferTests() {
        PoseBuffer poseBuffer = new PoseBuffer();
        for (int i = 0; i < 100; i++) {
            Pose2d pose2d = new Pose2d(Math.random() * 100, Math.random() * 100);
            double time = Math.random() * 10800;
            poseBuffer.pushToBuffer(pose2d);
        }
    }
}
