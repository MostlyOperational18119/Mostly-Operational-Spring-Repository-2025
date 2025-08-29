package org.firstinspires.ftc.teamcode;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;

public class BetterLimelight3A implements Runnable { // Not really necessary
    private final String DefaultIP = "192.168.43.2"; // Find the correct one :D
    private final int DefaultPort = 8888; // Find the correct one :D

    private Socket clientSocket;
    private InputStream inputStream;
    private OutputStream outputStream;
    private Boolean running = false;

    public BetterLimelight3A() throws IOException {
        connect();
    }

    public void connect() throws IOException {
        clientSocket = new Socket(DefaultIP, DefaultPort);

        inputStream = clientSocket.getInputStream();
        outputStream = clientSocket.getOutputStream();

        running = true;
    }

    public void disconnect() throws IOException {
        inputStream.close();
        outputStream.close();

        clientSocket.close();

        running = false;
    }

    @Override
    public void run() {
        while (running) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }


        }
    }
}
