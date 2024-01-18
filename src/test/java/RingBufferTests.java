import static org.junit.Assert.*;
import org.junit.jupiter.api.Test;


import frc.robot.utils.RingBuffer;

import java.util.NoSuchElementException;

public class RingBufferTests {

    @Test
    public void populateAndRandomAccess() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addLast(5); // [5]
        buffer.addLast(6); // [5, 6]
        buffer.addLast(7); // [5, 6, 7]

        Assert.assertEquals(5, (int)buffer.getFromFirst(0));
        Assert.assertEquals(6, (int)buffer.getFromFirst(1));
        Assert.assertEquals(7, (int)buffer.getFromFirst(2));
    }

    @Test
    public void populateAndRandomAccessFromLast() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addLast(5); // [5];
        buffer.addLast(6); // [5, 6]
        buffer.addLast(7); // [5, 6, 7]

        Assert.assertEquals(7, (int)buffer.getFromLast(0));
        Assert.assertEquals(6, (int)buffer.getFromLast(1));
        Assert.assertEquals(5, (int)buffer.getFromLast(2));
    }

    @Test
    public void populateFromFirst() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addFirst(5); // [5]
        buffer.addFirst(6); // [6, 5]
        buffer.addFirst(7); // [7, 6, 5]

        Assert.assertEquals(5, (int)buffer.getFromLast(0));
        Assert.assertEquals(6, (int)buffer.getFromLast(1));
        Assert.assertEquals(7, (int)buffer.getFromLast(2));

        Assert.assertEquals(7, (int)buffer.getFromFirst(0));
        Assert.assertEquals(6, (int)buffer.getFromFirst(1));
        Assert.assertEquals(5, (int)buffer.getFromFirst(2));
    }

    @Test
    public void populateFromBothEnds() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addFirst(6); // [6]
        buffer.addFirst(5); // [5, 6]
        buffer.addLast(7);  // [5, 6, 7]
        buffer.addLast(8);  // [5, 6, 7, 8]

        Assert.assertEquals(8, (int)buffer.getFromLast(0));
        Assert.assertEquals(7, (int)buffer.getFromLast(1));
        Assert.assertEquals(6, (int)buffer.getFromLast(2));
        Assert.assertEquals(5, (int)buffer.getFromLast(3));

        Assert.assertEquals(5, (int)buffer.getFromFirst(0));
        Assert.assertEquals(6, (int)buffer.getFromFirst(1));
        Assert.assertEquals(7, (int)buffer.getFromFirst(2));
        Assert.assertEquals(8, (int)buffer.getFromFirst(3));
    }

    @Test
    public void outOfBoundsAccessThrows() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addLast(5); // [5]
        buffer.addLast(6); // [5, 6]
        buffer.addLast(7); // [5, 6, 7]

        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromFirst(3));
        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromFirst(4));
        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromFirst(5));
        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromLast(3));
        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromLast(4));
        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromLast(5));

        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromFirst(-1));
        Assert.assertThrows(NoSuchElementException.class, () -> buffer.getFromLast(-1));
    }

    @Test
    public void removingWhenEmptyThrows() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addLast(5); // [5]
        buffer.addLast(6); // [5, 6]
        buffer.addLast(7); // [5, 6, 7]

        buffer.removeFirst();
        buffer.removeFirst();
        buffer.removeFirst();

        Assert.assertThrows(NoSuchElementException.class, buffer::removeFirst);
        Assert.assertThrows(NoSuchElementException.class, buffer::removeLast);
    }

    @Test
    public void populateAndRemove() {
        RingBuffer<Integer> buffer = new RingBuffer<>();
        buffer.addLast(5); // [5]
        buffer.addLast(6); // [5, 6]
        buffer.addLast(7); // [5, 6, 7]

        buffer.removeFirst();   // Should remove the 5

        Assert.assertEquals(6, (int)buffer.getFromFirst(0));
        Assert.assertEquals(7, (int)buffer.getFromFirst(1));

        Assert.assertEquals(6, (int)buffer.getFromLast(1));
        Assert.assertEquals(7, (int)buffer.getFromLast(0));
    }

    @Test
    public void populateAndRemoveManyTimes() {
        RingBuffer<Integer> buffer = new RingBuffer<>();

        // This should result in the buffer always containing between 3 and 5 elements.
        buffer.addLast(-1);
        buffer.addLast(-1);
        buffer.addLast(-1);
        for (int i=0; i < 100; i+=2) {
            buffer.addLast(i);
            buffer.addLast(i+1);
            buffer.removeFirst();
            buffer.removeFirst();
        }

        // Check that the buffer has only 3 elements
        Assert.assertEquals(3, buffer.size());
        // Check that these elements are [97, 98, 99]
        Assert.assertEquals(97, (int)buffer.getFromFirst(0));
        Assert.assertEquals(98, (int)buffer.getFromFirst(1));
        Assert.assertEquals(99, (int)buffer.getFromFirst(2));

        // Check that the buffer has not grown in capacity to some absurd size
        // Will use 8 since my implementation will probably grow by doubling the backing array size.
        Assert.assertTrue(buffer.capacity() <= 8);
    }
}