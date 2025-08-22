package org.firstinspires.ftc.teamcode;

import static java.util.Arrays.stream;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

// Better LimeLight Results (like LLResults but better :D )

public class BLLResults {
    public enum BLLResultType {

    }

    private final byte[] ToLimelightMagic = {0x43, 0x4C, 0x49, 0x45, 0x4E, 0x54, 0x00};
    private final byte[] ToRobotMagic = {0x4C, 0x53, 0x52, 0x56, 0x00};

    public Boolean valid;

    public BLLResults(byte[] data) {
        ByteBuffer buffer = ByteBuffer.wrap(data);


        this.valid = Arrays.equals(Arrays.copyOf(data, ToRobotMagic.length), ToRobotMagic);
        byte type = buffer.getChar(5);

        switch (type) {
            case 0x00:

            default:
                this.valid = false;
        }



    }
}
