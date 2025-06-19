package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    // 座標系們
    Map<String, Frame> frames;
    ObjectDetector objectDetector;

    @Override
    protected void runPlan1() {
        Frame frame;
        Image image;
        objectDetector = new YOLOSharedObjectDetector(this);

        // 座標系對應
        frames = new HashMap<>();
        // area1: 區域1 - 區域正中心、機器人要面向區域的方向
        frames.put("area1", new Frame(new Vector(10.950d, -10.580d, 5.195d), new Quater(0f,0f,-0.707f,0.707f)));
        // axis1: 軸向1 - 區域的法單位向量，代表機器人要加上這方向才能相對於區域後退
        frames.put("axis1", new Frame(new Vector(0d, 1.0d, 0d), new Quater(0f,0f,0f,1.0f)));
        // area2: 區域2 - 區域正中心、機器人要面向區域的方向
        frames.put("area2", new Frame(new Vector(10.925d, -8.875d, 3.76203d), new Quater(0f,0.707f,0f,0.707f)));
        // axis2: 軸向2 - 區域的法單位向量，代表機器人要加上這方向才能相對於區域後退
        frames.put("axis2", new Frame(new Vector(0d, 0d, 1.0d), new Quater(0f,0f,0f,1.0f)));
        // area3: 區域3 - 區域正中心、機器人要面向區域的方向
        frames.put("area3", new Frame(new Vector(10.925d, -7.925d, 3.76203d), new Quater(0f,0.707f,0f,0.707f)));
        // axis3: 軸向3 - 區域的法單位向量，代表機器人要加上這方向才能相對於區域後退
        frames.put("axis3", new Frame(new Vector(0d, 0d, 1.0d), new Quater(0f,0f,0f,1.0f)));
        // area4: 區域4 - 區域正中心、機器人要面向區域的方向
        frames.put("area4", new Frame(new Vector(9.866984d, -6.8525d, 4.945d), new Quater(0f,0f,1.0f,0f)));
        // axis4: 軸向4 - 區域的法單位向量，代表機器人要加上這方向才能相對於區域後退
        frames.put("axis4", new Frame(new Vector(1.0d, 0d, 0d), new Quater(0f,0f,0f,1.0f)));
        // astronaut: 太空人、機器人要面向太空人的方向
        frames.put("astronaut", new Frame(new Vector(11.143d, -6.7607d, 4.9654d), new Quater(0f,0f,0.707f,0.707f)));
        // way0: 主要幹線 - 起始點
        frames.put("way0", new Frame(new Vector(9.815d,-9.806d,4.293d), new Quater(1.0f,0f,0f,0f)));
        // way1: 主要幹線 - 幹線 1（可當錨點）
        frames.put("way1", new Frame(new Vector(10.925d, -9.806d, 4.945d), new Quater(0f,0f,0.707f,0.707f)));
        // way2: 主要幹線 - 幹線 2
        frames.put("way2", new Frame(new Vector(10.925d, -8.938d, 4.945d), new Quater(0f,0f,0.707f,0.707f)));
        // way3: 主要幹線 - 幹線 3
        frames.put("way3", new Frame(new Vector(10.925d, -8.071d, 4.945d), new Quater(0f,0f,0.707f,0.707f)));
        // way4: 主要幹線 - 幹線 4
        frames.put("way4", new Frame(new Vector(10.925d, -7.200d, 4.945d), new Quater(0f,0f,0.707f,0.707f)));

        // The mission starts.
        api.startMission();

        /* ******************************************************************************** */
        /* CCSH Function Test                                                               */
        /* ******************************************************************************** */

//        // 校正
//        PIDController<Vector> posPID = new PIDController<>(5.0, 0.01, 0.0);
//        PIDController<Quater> oriPID = new PIDController<>(5.0, 0.01, 0.0);
//        posPID.setSetpoint(frames.get("way1").getPosition());
//        oriPID.setSetpoint(frames.get("way1").getOrientation());
//        Frame errorFrame = frames.get("way1").anchor(api, 7);

        // Get a camera image.

        image = Image.undistort(api);
        image.save("start.png");

        List<AreaInfo> areaInfos = new ArrayList<>();
        areaInfos.add(round(1));
        areaInfos.add(round(2));
        areaInfos.add(round(3));
        areaInfos.add(round(4));

        /* ******************************************************************************** */
        /* Write your code to recognize the type and number of landmark items in each area! */
        /* If there is a treasure item, remember it.                                        */
        /* ******************************************************************************** */

        // When you recognize landmark items, let’s set the type and number.
        // api.setAreaInfo(1, "item_name", 1);

        /* ************************************************ */
        /* Let's move to each area and recognize the items. */
        /* ************************************************ */

        // When you move to the front of the astronaut, report the rounding completion.
        ItemInfo targetItem = astronaut();

        /* ***************************************************************** */
        /* Write your code to recognize which target item the astronaut has. */
        /* ***************************************************************** */

        // Let's notify the astronaut when you recognize it.
        // api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        // api.takeTargetItemSnapshot();

        findTarget(areaInfos, targetItem);
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

    AreaInfo round(int areaId) {
        String area = "area" + areaId;
        String axis = "axis" + areaId;

        Frame frame = frames.get(area).absolute(frames.get(axis).gain(1.0));
        frame.moveTo(api, false);
        frame = Image.anchor(api, frames.get(axis), area, 3);
        Frame location = frame.absolute(frames.get(axis).gain(-0.3)); // 儲存距離 0.7m 的位置，之後就不用調了
        Image image = Image.undistort(api);
        image.save(area + ".png");
        ArucoResult arucoResult = image.aruco(area);
        Image paper = image.correctA4Paper(area);
        Image region = image.crop(area);
        if (region != null) region.save(area + "_crop.png");

        List<ItemInfo> items = new ArrayList<>();
        try {
            // 步驟 1: 執行檢測並取得結果 Map
            items = objectDetector.detect(region.getMatImage(), "lost");

            // 步驟 2: 顯示結果
            if (items != null && !items.isEmpty()) {
                StringBuilder resultBuilder = new StringBuilder("檢測結果:\n");
                for (ItemInfo item: items) {
                    resultBuilder.append(item.getName()).append(": ").append(item.getNumber()).append(" 個\n");
                    if (!item.getName().equals("crystal") && !item.getName().equals("diamond") && !item.getName().equals("emerald")) {
                        api.setAreaInfo(areaId, item.getName(), item.getNumber());
                    }
                }
                Log.i("Object_Detection", resultBuilder.toString());
            } else {
                Log.i("Object_Detection", "未檢測到物件。");
            }

        } catch (Exception e) {
            Log.e("Object_Detection", "檢測過程發生錯誤: " + e.getMessage(), e);
        }

        return new AreaInfo(areaId, location, region, paper, arucoResult, items);
    }

    ItemInfo astronaut() {
        try {
            // When you move to the front of the astronaut, report the rounding completion.
            frames.get("astronaut").moveTo(api, false);
            Log.i("ccsh", "astronaut: " + new Frame(api));
            api.reportRoundingCompletion();

            frames.get("astronaut").anchor(api, 10); // wait for about 10 seconds

            Image image = Image.undistort(api);
            image.save("astronaut.png");
            Image region = image.crop("astronaut");
            if (region != null) region.save("astronaut_crop.png");

            ItemInfo targetItem = null;
            List<ItemInfo> items = objectDetector.detect(region.getMatImage(), "target");
            if (items != null && !items.isEmpty()) {
                StringBuilder resultBuilder = new StringBuilder("檢測結果:\n");
                for (ItemInfo item: items) {
                    resultBuilder.append(item.getName()).append(": ").append(item.getNumber()).append(" 個\n");
                    if (item.getName().equals("crystal") || item.getName().equals("diamond") || item.getName().equals("emerald")) {
                        targetItem = item;
                        break;
                    }
                }
                Log.i("Object_Detection", resultBuilder.toString());
            } else {
                Log.i("Object_Detection", "未檢測到物件。");
            }

            return targetItem;

        } catch (Exception e) {
            Log.e("Astronaut", "太空人發生錯誤: " + e.getMessage(), e);
        }

        return null;
    }

    void findTarget(List<AreaInfo> areaInfos, ItemInfo targetItem) {
        if (targetItem == null) return;

        AreaInfo targetArea = null;
        for (AreaInfo areaInfo: areaInfos) {
            for (ItemInfo item: areaInfo.getItems()) {
                if (item.getName().equals(targetItem.getName())) {
                    targetArea = areaInfo;
                    break;
                }
            }
        }

        if (targetArea != null) {
            targetArea.getLocation().moveTo(api, true);

            // Let's notify the astronaut when you recognize it.
            api.notifyRecognitionItem();

            // Take a snapshot of the target item.
            Image image = Image.undistort(api);
            image.save("target.png");
            api.takeTargetItemSnapshot();
        }
    }
}
