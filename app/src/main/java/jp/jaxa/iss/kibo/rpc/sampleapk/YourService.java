package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.provider.ContactsContract;
import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;

import java.util.Observer;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        long startTime = System.currentTimeMillis();

        Kinematics kinematics;
        Vector position;
        Quater orientation;
        Frame frame;

        // 世界座標
        Frame worldFrame = new Frame(
            new Vector(9.815,-9.806,4.293),
            new Quater(1,0,0,0)
        );

        // The mission starts.
        api.startMission();

        // 校正機器感知座標用（相對於世界座標）
        Frame errorRobotFrame = worldFrame.relative(api.getRobotKinematics());
        // 校正機器移動座標用（相對於世界座標）
        Frame errorMoveToFrame;

        Vector v, a, w;
        do {
            kinematics = api.getRobotKinematics();
            v = new Vector(kinematics.getLinearVelocity());
            a = new Vector(kinematics.getLinearAcceleration());
            w = new Vector(kinematics.getAngularVelocity());
            Log.i("jerry", "v: " + v + ", a: " + a + ", w: " + w);
            frame = errorRobotFrame.absolute(kinematics);
            errorMoveToFrame = frame.relative(worldFrame);
            Log.i("jerry", "p: " + errorMoveToFrame.getPosition() + ", o: " + errorMoveToFrame.getOrientation());
            api.moveTo(worldFrame.getPosition(), worldFrame.getOrientation(), true);
        } while (v.norm() > 0.002 || a.norm() > 0.002 || w.norm() > 0.002);

        long endTime = System.currentTimeMillis();

        Log.i("jerry_time", "time: " + (endTime - startTime) / 1000 + " seconds.");

        for (int i = 0; i < 100; i++) {
            frame = errorMoveToFrame.absolute(worldFrame);
            api.moveTo(frame.getPosition(), frame.getOrientation(), true);

            kinematics = api.getRobotKinematics();
            Frame deltaFrame = errorMoveToFrame.absolute(kinematics).relative(worldFrame);
            Log.i("jerry", "v: " + kinematics.getLinearVelocity() + ", a: " + kinematics.getLinearAcceleration() + ", w: " + kinematics.getAngularVelocity());
            Log.i("jerry", "p: " + deltaFrame.getPosition() + ", o: " + deltaFrame.getOrientation());
        }

        kinematics = api.getRobotKinematics();
        Log.i("jerry", "x: " + kinematics.getPosition() + ", r: " + kinematics.getOrientation());
        waitForSeconds(5);
        kinematics = api.getRobotKinematics();
        Log.i("jerry.wait", "x: " + kinematics.getPosition() + ", r: " + kinematics.getOrientation());
        // Move to a point.
        position = new Vector(10.9d, -9.92284d, 5.195d);
        orientation = new Quater(0f, 0f, -0.707f, 0.707f);
        api.moveTo(position, orientation, false);

        kinematics = api.getRobotKinematics();
        Log.i("jerry1", "x: " + kinematics.getPosition() + ", r: " + kinematics.getOrientation());


        // Get a camera image.
        Mat image = api.getMatNavCam();

        /* ******************************************************************************** */
        /* Write your code to recognize the type and number of landmark items in each area! */
        /* If there is a treasure item, remember it.                                        */
        /* ******************************************************************************** */

        // When you recognize landmark items, let’s set the type and number.
        api.setAreaInfo(1, "item_name", 1);

        /* **************************************************** */
        /* Let's move to each area and recognize the items. */
        /* **************************************************** */

        // When you move to the front of the astronaut, report the rounding completion.
        position = new Vector(11.143d, -6.7607d, 4.9654d);
        orientation = new Quater(0f, 0f, 0.707f, 0.707f);
        api.moveTo(position, orientation, false);
        kinematics = api.getRobotKinematics();
        Log.i("jerry2", "x: " + kinematics.getPosition() + ", r: " + kinematics.getOrientation());

        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which target item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
       // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // You can add your method.
    private String yourMethod(){
        return "your method";
    }

    // Helper method to wait for specified seconds
    private void waitForSeconds(int seconds) {
        try {
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            // Handle interruption if necessary
            e.printStackTrace();
        }
    }
}
