import java.awt.image.DataBufferByte;
import org.opencv.core.Point;
import org.opencv.highgui.*;
import org.opencv.imgproc.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc.*;
import org.opencv.highgui.VideoCapture;
import org.opencv.video.Video;
import java.net.Socket;
import java.util.ArrayList;
import java.awt.image.*;
import static java.lang.Math.tan;
import static org.opencv.imgproc.Imgproc.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.xml.crypto.Data;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.Arrays;

public class SimpleSample {

    static{ System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }
    double targetHeight = 68.25;
    double cameraHeight = 14.5;
    double verticalFOV = 35;
    double horizontalFOV = 60;
    double cameraAngle = 49;
    double pi = 3.141592;

    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    ArrayList<Rect> rects = new ArrayList<Rect>();
    ArrayList<Rect> goodRects = new ArrayList<Rect>();
    ArrayList<Rect> myFavoriteRects = new ArrayList<Rect>();
    MatOfPoint2f contour2;
    MatOfPoint2f approxContours;
    VideoCapture gearCamera;
    VideoCapture boilerCamera;
    RoboRIOConnection connection;
    Imshow im;
    enum GearVisionMode  {VisionTargeting, DriverOutput, None};
    public static GearVisionMode wantedGearVisionMode;
    enum BoilerVisionMode  {VisionTargeting, DriverOutput, None};
    public static BoilerVisionMode wantedBoilerVisionMode;
    double closestRect ;
    double y;
    double x;
    double distance;
    boolean gearTargetTracked = false;
    double great;
    Mat input = new Mat();
    Mat blurred = new Mat();
    Mat HSVImage = new Mat();
    Mat thresholded = new Mat();
    Mat reading = new Mat();
    double azimuth = 0;
    boolean shouldRun;
    Thread visionThread;
    long timeSinceLastMessage;
    String inputLine;
    Thread m_Thread;
    Thread visionCommsThread;
    Socket dataCommSocket;
    BufferedReader dataCommInput;
    DataOutputStream dataCommOutput;
    Socket driverStationVisionSocket;
    BufferedReader driverStationVisionInput;
    DataOutputStream driverStationVisionOutput;
    public SimpleSample() {
        wantedGearVisionMode = GearVisionMode.None;
        wantedBoilerVisionMode = BoilerVisionMode.None;
        wantedBoilerVisionMode = BoilerVisionMode.None;
        wantedGearVisionMode = GearVisionMode.VisionTargeting;
        connection = new RoboRIOConnection();
        connection.start();
  //      im = new Imshow("Video Preview");
  //      im.Window.setResizable(true);
        gearCamera = new VideoCapture(0);
        while (gearCamera.isOpened() == false) {

         }
         gearCamera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, 320);
        gearCamera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, 240);
        while (reading.empty()) {
            gearCamera.retrieve(reading);
        }
//        boilerCamera = new VideoCapture(2);
//        reading = new Mat();
//        while (boilerCamera.isOpened() == false) {
//
//        }
//        while (reading.empty()) {
//            boilerCamera.retrieve(reading);
//        }
        visionThread = new Thread(new RunVision());
        visionThread.start();
    }
    public void switchToModeGearAutoTargeting(){
        wantedGearVisionMode = GearVisionMode.VisionTargeting;
        Runtime r = Runtime.getRuntime();
        Process p = null;
        try {
            p = r.exec("v4l2-ctl --set-ctrl=exposure_auto=1 -d /dev/video0");
            p.waitFor();
            p = r.exec("v4l2-ctl --set-ctrl=exposure_absolute=5 -d /dev/video0");
            p.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void switchToModeGearDriverVision(){
        wantedGearVisionMode = GearVisionMode.DriverOutput;
        Runtime r = Runtime.getRuntime();
        Process p = null;
        try {
            p = r.exec("v4l2-ctl --set-ctrl=exposure_auto=1 -d /dev/video0");
            p.waitFor();
            p = r.exec("v4l2-ctl --set-ctrl=exposure_absolute=156 -d /dev/video0");
            p.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void switchToModeBoilerAutoTargeting(){
        wantedGearVisionMode = GearVisionMode.VisionTargeting;
        Runtime r = Runtime.getRuntime();
        Process p = null;
        try {
            p = r.exec("v4l2-ctl --set-ctrl=exposure_auto=1 -d /dev/video2");
            p.waitFor();
            p = r.exec("v4l2-ctl --set-ctrl=exposure_absolute=5 -d /dev/video2");
            p.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void switchToModeBoilerDriverVision(){
        wantedGearVisionMode = GearVisionMode.DriverOutput;
        Runtime r = Runtime.getRuntime();
        Process p = null;
        try {
            p = r.exec("v4l2-ctl --set-ctrl=exposure_auto=1 -d /dev/video2");
            p.waitFor();
            p = r.exec("v4l2-ctl --set-ctrl=exposure_absolute=156 -d /dev/video2");
            p.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void processGearAutoTarget() {
        while (wantedGearVisionMode == GearVisionMode.VisionTargeting) {
            try {
                closestRect = 1080;
                myFavoriteRects.clear();
                goodRects.clear();
                contours.clear();
                rects.clear();
                gearCamera.read(input);
                if (input.empty()) {
                    return;
                }
                GaussianBlur(input, blurred, new Size(5, 5), 0, 0);
                cvtColor(blurred, HSVImage, COLOR_BGR2HSV);//convert to HSV
                Core.inRange(HSVImage, new Scalar(55, 180, 75), new Scalar(110, 255, 185), thresholded);
                Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                for (int i = 0; i < contours.size(); i++) {
                    //Imgproc.drawContours(input, contours, i, new Scalar(4, 4, 255));
                    rects.add(Imgproc.boundingRect(contours.get(i)));
                    if (rects.get(i).height > 25 && rects.get(i).width > 7) {
                        contour2 = new MatOfPoint2f();
                        approxContours = new MatOfPoint2f();
                        contour2.fromArray(contours.get(i).toArray());
                        Imgproc.approxPolyDP(contour2, approxContours, 6, true);
                        if (approxContours.total() == 4) {
                            Core.rectangle(input, new Point(rects.get(i).x, rects.get(i).y), new Point(rects.get(i).x + rects.get(i).width, rects.get(i).y + rects.get(i).height), new Scalar(255, 0, 0));
                            goodRects.add(rects.get(i));
                        }
                    }

                }
                double ydiff = 0;
                //System.out.println(goodRects.size());
                for (int i = 0; i < goodRects.size(); i++) {
                    for (int n = 0; n < goodRects.size(); n++) {
                        if(i != n) {
                            ydiff = Math.abs(goodRects.get(i).br().y - goodRects.get(n).br().y);
                            if (ydiff < closestRect) {
                                myFavoriteRects.clear();
                                closestRect = ydiff;
                                myFavoriteRects.add(goodRects.get(i));
                                myFavoriteRects.add(goodRects.get(n));
                            }
                        }
                    }
                }
                if(closestRect < 1080) {
                    azimuth = (Math.abs((((myFavoriteRects.get(0).br().x - myFavoriteRects.get(0).width / 2) + (myFavoriteRects.get(1).br().x + myFavoriteRects.get(1).width / 2)) / 2) / gearCamera.get(Highgui.CV_CAP_PROP_FRAME_WIDTH) * 60) - 30);
                    System.out.println(azimuth);
                    gearTargetTracked = true;
                } else {
                    gearTargetTracked = false;
                }
      //          im.showImage(input);
            }catch(Exception e) {
                e.printStackTrace();
            }
        }
    }
    public void processGearDriverVision() {
        double[] bytesOfMat;
        double[] bytesToSend;
        gearCamera.read(input);
        bytesOfMat = new double[(int) (input.total() * input.channels())];
        while (wantedGearVisionMode == GearVisionMode.DriverOutput) {
            gearCamera.read(input);
            output = input.clone();
            if (input.empty()) {
                return;
            }
            input.convertTo(input, CvType.CV_64FC3);
            input.get(0, 0, bytesOfMat);
            bytesToSend = bytesOfMat.clone();
            try {
                System.out.println((int) (input.total() * input.channels()));
                //System.out.println(Arrays.toString(bytesToSend));
                System.out.println("dsdasdsdasadssda");
                //driverStationVisionOutput.writeBytes( createMessageTypeTag(Arrays.toString(bytesToSend)) + '\n');
                if (!driverStationVisionSocket.isClosed()) {
                    System.out.println("dsdasdssdsadsdasdasdasdasdasdadsasdasdadasadssda");

                    System.out.println("dsdasdsdasadssda");

                }
            } catch (Exception e) {
                e.printStackTrace();
            }
            im.showImage(output);
        }
    }
    public void processVisionModeRequest(String message){
        String visionMode;
        visionMode = decodeMessage("VisionModeRequest", message);
        if(visionMode.equals("GearAutoTarget")){
            System.out.println("Gear auto targeting Requested");
            wantedBoilerVisionMode = BoilerVisionMode.None;
            wantedGearVisionMode = GearVisionMode.VisionTargeting;
        } else if(visionMode.equals("GearDriverVision")){
            System.out.println("Gear Driver Vision Requested");
            wantedBoilerVisionMode = BoilerVisionMode.None;
            wantedGearVisionMode = GearVisionMode.DriverOutput;
        } else if(visionMode.equals("BoilerAutoTarget")){
            wantedGearVisionMode = GearVisionMode.None;
            wantedBoilerVisionMode = BoilerVisionMode.VisionTargeting;
        } else if (visionMode.equals("BoilderDriverVision")){
            wantedBoilerVisionMode = BoilerVisionMode.DriverOutput;
            wantedGearVisionMode = GearVisionMode.None;
        }
    }
    public String decodeMessage(String tag, String message){
        int start;
        int stop;
        int tagLength;
        start = message.indexOf("<" + tag + ">");
        stop = message.indexOf("</" + tag + ">");
        tagLength = ("<" + tag + ">").length();
        if(start >= 0 && stop >= 0 && start + tagLength >= 0){
            return message.substring(start + tagLength, stop);
        } else {
            return "error";
        }
    }
    public void send(String message){
        try {
            dataCommOutput.writeBytes(message + '\n');
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    public boolean containsTag(String tag, String message){
        return message.contains("<" + tag + ">") && message.contains("</" + tag + ">");
    }
    public String createMessageTypeTag(String messageType) {
        return createTaggedMessage("DriverStationImage", messageType);
    }
    public String createKnockKnockMessage() {
        return createTaggedMessage("KnockKnock","Knock Knock Request");
    }
    public String createTargetingMessage() {
        return createTaggedMessage("TargetInfo", ((Double)azimuth).toString()) + createTaggedMessage("TargetTracked", ((Boolean)gearTargetTracked).toString());
    }
    public String createTaggedMessage(String tag, String message) {
        return "<" + tag + ">" + message + "</" + tag + ">";
    }
    public void processBoilerAutoTarget() {
        while (wantedGearVisionMode == GearVisionMode.VisionTargeting) {
            try {
                closestRect = 1080;
                myFavoriteRects.clear();
                goodRects.clear();
                contours.clear();
                rects.clear();
                gearCamera.read(input);
                if (input.empty()) {
                    return;
                }
                GaussianBlur(input, blurred, new Size(5, 5), 0, 0);
                cvtColor(blurred, HSVImage, COLOR_BGR2HSV);//convert to HSV
                Core.inRange(HSVImage, new Scalar(0, 40, 75), new Scalar(110, 255, 185), thresholded);
                Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                for (int i = 0; i < contours.size(); i++) {
                    //Imgproc.drawContours(input, contours, i, new Scalar(4, 4, 255));
                    rects.add(Imgproc.boundingRect(contours.get(i)));
                    if (rects.get(i).height > 30 && rects.get(i).width > 7) {
                        contour2 = new MatOfPoint2f();
                        approxContours = new MatOfPoint2f();
                        contour2.fromArray(contours.get(i).toArray());
                        Imgproc.approxPolyDP(contour2, approxContours, 6, true);
                        if (approxContours.total() == 4) {
                            Core.rectangle(input, new Point(rects.get(i).x, rects.get(i).y), new Point(rects.get(i).x + rects.get(i).width, rects.get(i).y + rects.get(i).height), new Scalar(255, 0, 0));
                            goodRects.add(rects.get(i));
                        }
                    }

                }
                double ydiff = 0;
                //System.out.println(goodRects.size());
                for (int i = 0; i < goodRects.size(); i++) {
                    for (int n = 0; n < goodRects.size(); n++) {
                        if(i != n) {
                            ydiff = Math.abs(goodRects.get(i).br().y - goodRects.get(n).br().y);
                            if (ydiff < closestRect) {
                                myFavoriteRects.clear();
                                closestRect = ydiff;
                                myFavoriteRects.add(goodRects.get(i));
                                myFavoriteRects.add(goodRects.get(n));
                            }
                        }
                    }
                }
                if(closestRect < 1080) {
                    azimuth = (Math.abs((((myFavoriteRects.get(0).br().x - myFavoriteRects.get(0).width / 2) + (myFavoriteRects.get(1).br().x + myFavoriteRects.get(1).width / 2)) / 2) / gearCamera.get(Highgui.CV_CAP_PROP_FRAME_WIDTH) * 60) - 30);
                    System.out.println(azimuth);
                }
                im.showImage(input);
            }catch(Exception e) {
                e.printStackTrace();
            }
        }
    }
    public void processBoilerDriverVision(){
//        output = new Mat();
//        while (wantedGearVisionMode == GearVisionMode.DriverOutput) {
//            gearCamera.read(input);
//            if (input.empty()) {
//                return;
//            }
//            matToSend = new BufferedImage(input.width(), input.height(), BufferedImage.TYPE_3BYTE_BGR);
//            bytesOfMat = ((DataBufferByte)matToSend.getRaster().getDataBuffer()).getData();
//            input.get(0, 0, bytesOfMat);
//            try {
//                System.out.println(new String(bytesOfMat));
//                outToServer.writeBytes(bytesOfMat.toString() + "Hello" + '\n');
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//            im.showImage(input);
//        }
    }
    BufferedImage matToSend;
    Mat output = new Mat();
    public static void main(String[] args) {
        SimpleSample test = new SimpleSample();
    }
    protected class RunVision implements Runnable{
        public void run() {
            //visionCommsThread = new Thread(new RunVisionComms());
            //visionCommsThread.start();

            while (true) {
                if (wantedBoilerVisionMode == BoilerVisionMode.None && wantedGearVisionMode == GearVisionMode.VisionTargeting) {
                    switchToModeGearAutoTargeting();
                    processGearAutoTarget();
                } else if (wantedBoilerVisionMode == BoilerVisionMode.None && wantedGearVisionMode == GearVisionMode.DriverOutput) {
                    switchToModeGearDriverVision();
                    //processGearDriverVision();
                } else if (wantedBoilerVisionMode == BoilerVisionMode.VisionTargeting && wantedGearVisionMode == GearVisionMode.None) {
                    switchToModeBoilerAutoTargeting();
                } else if (wantedBoilerVisionMode == BoilerVisionMode.DriverOutput && wantedGearVisionMode == GearVisionMode.None) {

                }
            }
        }
    }
    protected class RunVisionComms implements Runnable{
        public RunVisionComms(){
                shouldRun = true;
            }
        public void run() {
            try{
                try{
                    driverStationVisionSocket = new Socket("10.3.60.109", 3601);
                    driverStationVisionInput = new BufferedReader(new InputStreamReader(driverStationVisionSocket.getInputStream()));
                    //driverStationVisionOutput = new DataOutputStream(driverStationVisionSocket.getOutputStream());
                    //objectOutput = new ObjectOutputStream(driverStationVisionSocket.getOutputStream());

                } catch (Exception e) {
                    //e.printStackTrace();
                }

            } catch (Exception e) {
                //e.printStackTrace();
            }
            while(driverStationVisionSocket != null && shouldRun && !driverStationVisionSocket.isClosed() && driverStationVisionSocket.isConnected() &&
                    System.currentTimeMillis() - timeSinceLastMessage < 333){
                try {
                    while(driverStationVisionInput.ready() && (inputLine = driverStationVisionInput.readLine()) != null){

                    }
                } catch (Exception e) {
                    System.out.println(e.toString());
                }
                try{
                //driverStationVisionOutput.writeBytes("dssadasdas" + '\n');
                } catch (Exception e) {
                    e.printStackTrace();
                }
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            try {
                driverStationVisionSocket.close();
            } catch (Exception e) {
                System.out.println("error " + e.toString());
            }
            System.out.println("connection done");
            if(shouldRun){
                try{
                    Thread.sleep(100);
                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                if(shouldRun){
                    run();
                }
            }
        }
        public void start() {
            System.out.println("Starting " );
            if (m_Thread == null) {
                m_Thread = new Thread (this);
                m_Thread.start ();
            }
        }
    }
    protected class RoboRIOConnection implements Runnable{

        public RoboRIOConnection(){
                shouldRun = true;
        }
        public void run() {
            try{
                dataCommSocket = new Socket("10.3.60.109", 3600);
                dataCommInput = new BufferedReader(new InputStreamReader(dataCommSocket.getInputStream()));
                dataCommOutput = new DataOutputStream(dataCommSocket.getOutputStream());
                send(createKnockKnockMessage());
                timeSinceLastMessage = System.currentTimeMillis();
            } catch (Exception e) {
                //e.printStackTrace();
            }
            while(dataCommSocket != null && shouldRun && !dataCommSocket.isClosed() && dataCommSocket.isConnected() &&
                    System.currentTimeMillis() - timeSinceLastMessage < 333){
                try {
                    while(dataCommInput.ready() && (inputLine = dataCommInput.readLine()) != null){
                        //System.out.println("received Message: " + inputLine);
                        if(containsTag("whosThere", inputLine)){
                            System.out.println("Whos There");
                            timeSinceLastMessage = System.currentTimeMillis();
                        } else if(containsTag("VisionModeRequest", inputLine)){
                            processVisionModeRequest(inputLine);
                        }
                    }
                } catch (Exception e) {
                    System.out.println(e.toString());
                }
                if(System.currentTimeMillis() - timeSinceLastMessage > 100){
                    send(createKnockKnockMessage());
                }
                try {
                    Thread.sleep(20);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                send(createTargetingMessage());
            }
            try {
                dataCommSocket.close();
            } catch (Exception e) {
                System.out.println("error " + e.toString());
            }
            System.out.println("connection done");
            if(shouldRun){
                try{
                    Thread.sleep(100);
                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                if(shouldRun){
                    run();
                }
            }
        }
        public void start() {
            System.out.println("Starting " );
            if (m_Thread == null) {
                m_Thread = new Thread (this);
                m_Thread.start ();
            }
        }
    }
}
// credit to @master-atul
class Imshow {

    public JFrame Window;
    private ImageIcon image;
    private JLabel label;
    // private MatOfByte matOfByte;
    private Boolean SizeCustom;
    private int Height, Width;

    public Imshow(String title) {
        Window = new JFrame();
        image = new ImageIcon();
        label = new JLabel();
        // matOfByte = new MatOfByte();
        label.setIcon(image);
        Window.getContentPane().add(label);
        Window.setResizable(false);
        Window.setTitle(title);
        SizeCustom = false;
        setCloseOption(0);
    }

    public Imshow(String title, int height, int width) {
        SizeCustom = true;
        Height = height;
        Width = width;

        Window = new JFrame();
        image = new ImageIcon();
        label = new JLabel();
        // matOfByte = new MatOfByte();
        label.setIcon(image);
        Window.getContentPane().add(label);
        Window.setResizable(false);
        Window.setTitle(title);
        setCloseOption(0);

    }

    public void showImage(Mat img) {
        if (SizeCustom) {
            Imgproc.resize(img, img, new Size(Height, Width));
        }
        // Highgui.imencode(".jpg", img, matOfByte);
        // byte[] byteArray = matOfByte.toArray();
        BufferedImage bufImage = null;
        try {
            // InputStream in = new ByteArrayInputStream(byteArray);
            // bufImage = ImageIO.read(in);
            bufImage = toBufferedImage(img);
            image.setImage(bufImage);
            Window.pack();
            label.updateUI();
            Window.setVisible(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // CREDITS TO DANIEL: http://danielbaggio.blogspot.com.br/ for the improved
    // version !

    public BufferedImage toBufferedImage(Mat m) {
        int type = BufferedImage.TYPE_BYTE_GRAY;
        if (m.channels() > 1) {
            type = BufferedImage.TYPE_3BYTE_BGR;
        }
        int bufferSize = m.channels() * m.cols() * m.rows();
        byte[] b = new byte[bufferSize];
        m.get(0, 0, b); // get all the pixels
        BufferedImage image = new BufferedImage(m.cols(), m.rows(), type);
        final byte[] targetPixels = ((DataBufferByte) image.getRaster()
                .getDataBuffer()).getData();
        System.arraycopy(b, 0, targetPixels, 0, b.length);
        return image;

    }

    // Thanks to sutr90 for reporting the issue : https://github.com/sutr90

    public void setCloseOption(int option) {

        switch (option) {
            case 0:
                Window.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
                break;
            case 1:
                Window.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
                break;
            default:
                Window.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        }

    }

    /**
     * Sets whether this window should be resizable or not, by default it is not
     * resizable
     *
     * @param resizable
     *            <code>true</code> if the window should be resizable,
     *            <code>false</code> otherwise
     */
    public void setResizable(boolean resizable) {
        Window.setResizable(resizable);
    }


    // Thanks to Jan Monterrubio for additional static methods for viewing images.


    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow}
     *
     * @param mat
     *            the {@link Mat} to display
     */
    public static void show(Mat mat) {
        show(mat, new Dimension(mat.rows(), mat.cols()), "", false,
                WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow} with
     * the given title as the title for the window
     *
     * @param mat
     *            the {@link Mat} to display
     * @param frameTitle
     *            the title for the frame
     */
    public static void show(Mat mat, String frameTitle) {
        show(mat, new Dimension(mat.rows(), mat.cols()), frameTitle, false,
                WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow} with
     * the given title as the title for the window and determines whether the
     * frame is resizable or not
     *
     * @param mat
     *            the {@link Mat} to display
     * @param frameTitle
     *            the title for the frame
     * @param resizable
     *            whether the frame should be resizable or not
     */
    public static void show(Mat mat, String frameTitle, boolean resizable) {
        show(mat, new Dimension(mat.rows(), mat.cols()), frameTitle, resizable,
                WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow} with a
     * set size
     *
     * @param mat
     *            the {@link Mat} to display
     * @param frameSize
     *            the size for the frame
     */
    public static void show(Mat mat, Dimension frameSize) {
        show(mat, frameSize, "", false, WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow} with a
     * set size and given title
     *
     * @param mat
     *            the {@link Mat} to display
     * @param frameSize
     *            the size for the frame
     * @param frameTitle
     *            the title for the frame
     */
    public static void show(Mat mat, Dimension frameSize, String frameTitle) {
        show(mat, frameSize, frameTitle, false, WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow} with a
     * set size and given title and whether it is resizable or not
     *
     * @param mat
     *            the {@link Mat} to display
     * @param frameSize
     *            the size for the frame
     * @param frameTitle
     *            the title for the frame
     */
    public static void show(Mat mat, Dimension frameSize, String frameTitle,
                            boolean resizable) {
        show(mat, frameSize, frameTitle, resizable,
                WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Displays the given {@link Mat} in a new instance of {@link Imshow} with a
     * set size and given title and whether it is resizable or not, and with the
     * close operation set
     *
     * @param mat
     *            the {@link Mat} to display
     * @param frameSize
     *            the size for the frame
     * @param frameTitle
     *            the title for the frame
     * @param resizable
     *            wether the frame is resizable or not
     * @param closeOperation
     *            the constant for the default close operation of the frame
     */
    public static void show(Mat mat, Dimension frameSize, String frameTitle,
                            boolean resizable, int closeOperation) {
        Imshow frame = new Imshow(frameTitle, frameSize.height, frameSize.width);
        frame.setResizable(resizable);

		/*
		 * This is a bad way to access the window, but due to legacy stuff I
		 * won't change the access patterns
		 */
        frame.Window.setDefaultCloseOperation(closeOperation);
        frame.showImage(mat);
    }

}

 /*     double targetHeight = 68.25;
        double cameraHeight = 14.5;
        double verticalFOV = 35;
        double horizontalFOV = 60;
        double cameraAngle = 49;
        double pi = 3.141592;

        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        ArrayList<MatOfPoint> approx1 = new ArrayList<MatOfPoint>();
        MatOfPoint approx2 = new MatOfPoint();
        double y;
        double x;
        double distance;
        double azimuth;
        double great;
        Rect rec;
        Mat image = new Mat();
        Mat frame = new Mat();
        Mat heir = new Mat();
        for(;;) {
//            great = 0;
//            approx1.clear();
//            approx2 = new MatOfPoint();
//            cap.read(image);
//            GaussianBlur(image, image, new Size(5, 5), 0, 0);
//            if (image.empty()) {
//                return;
//            }
//            cvtColor(image, image, COLOR_BGR2HSV);convert to HSV
//            Core.inRange(image, new Scalar(55, 180, 75), new Scalar(110, 255, 185), frame);
            Imgproc.findContours(frame, contours, heir, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		    System.out.println("hi");
            for (int i = 0; i < contours.size(); i++) {
                 rec = Imgproc.boundingRect(contours.get(i));
                if (rec.height > 25 && rec.width > 25) {
                    MatOfPoint2f otherContour = new MatOfPoint2f();
                    otherContour.fromArray((contours.get(i).toArray()));
                        great = rec.area();
                        approx1.add(approx2);
                        x = rec.br().x - rec.width / 2;
                        y = rec.br().y + rec.height / 2;
                        y = -((2 * (y / cap.get(CV_CAP_PROP_FRAME_HEIGHT))) - 1);
                        distance = (targetHeight - cameraHeight) / tan(((y * verticalFOV / 2 + cameraAngle) * pi / 180));
                        azimuth = (x / cap.get(CV_CAP_PROP_FRAME_WIDTH)) * horizontalFOV;
                        System.out.println(distance + " " + azimuth + " " + rec.br().x + " " + x + " " + rec.width);
                }
            }
        }
    }
}*/