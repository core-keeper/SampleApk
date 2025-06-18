package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;

import java.text.DecimalFormat;
import java.util.List;
import java.util.ArrayList; // Good practice to initialize lists

public class AreaInfo {
    private int id; // areaId
    private Frame location; // position, orientation
    private Image region; // 224 x 224 px image
    private Image paper; // A4 image
    private ArucoResult arucoResult; // Aruco tags
    private List<ItemInfo> items; // landmarks, treasures

    /**
     * Constructs a new AreaInfo object with the specified details.
     *
     * @param id The unique identifier for the area.
     * @param location The frame representing the position and orientation of the area.
     * @param region The 224x224 pixel image representing the region.
     * @param paper The A4 image associated with the area.
     * @param arucoResult The Aruco tags detected within the area.
     * @param items A list of items (landmarks, treasures) found in the area.
     */
    public AreaInfo(int id, Frame location, Image region, Image paper, ArucoResult arucoResult, List<ItemInfo> items) {
        this.id = id;
        this.location = location;
        this.region = region;
        this.paper = paper;
        this.arucoResult = arucoResult;
        this.items = items != null ? new ArrayList<ItemInfo>(items) : new ArrayList<ItemInfo>(); // Defensive copy
    }

    /**
     * Gets the unique identifier of the area.
     * @return The area ID.
     */
    public int getId() {
        return id;
    }

    /**
     * Sets the unique identifier of the area.
     * @param id The new area ID.
     */
    public void setId(int id) {
        this.id = id;
    }

    /**
     * Gets the frame representing the position and orientation of the area.
     * @return The location frame.
     */
    public Frame getLocation() {
        return location;
    }

    /**
     * Sets the frame representing the position and orientation of the area.
     * @param location The new location frame.
     */
    public void setLocation(Frame location) {
        this.location = location;
    }

    /**
     * Gets the 224x224 pixel image representing the region.
     * @return The region image.
     */
    public Image getRegion() {
        return region;
    }

    /**
     * Sets the 224x224 pixel image representing the region.
     * @param region The new region image.
     */
    public void setRegion(Image region) {
        this.region = region;
    }

    /**
     * Gets the A4 image associated with the area.
     * @return The paper image.
     */
    public Image getPaper() {
        return paper;
    }

    /**
     * Sets the A4 image associated with the area.
     * @param paper The new paper image.
     */
    public void setPaper(Image paper) {
        this.paper = paper;
    }

    /**
     * Gets the Aruco tags detected within the area.
     * @return The ArucoResult object.
     */
    public ArucoResult getArucoResult() {
        return arucoResult;
    }

    /**
     * Sets the Aruco tags detected within the area.
     * @param arucoResult The new ArucoResult object.
     */
    public void setArucoResult(ArucoResult arucoResult) {
        this.arucoResult = arucoResult;
    }

    /**
     * Gets the list of items (landmarks, treasures) found in the area.
     * @return A list of ItemInfo objects.
     */
    public List<ItemInfo> getItems() {
        return new ArrayList<>(items); // Return a defensive copy
    }

    /**
     * Sets the list of items (landmarks, treasures) found in the area.
     * @param items The new list of ItemInfo objects.
     */
    public void setItems(List<ItemInfo> items) {
        this.items = items != null ? new ArrayList<ItemInfo>(items) : new ArrayList<ItemInfo>(); // Defensive copy
    }

    /**
     * Adds an item to the list of items in the area.
     * @param item The ItemInfo object to add.
     */
    public void addItem(ItemInfo item) {
        if (this.items == null) {
            this.items = new ArrayList<>();
        }
        this.items.add(item);
    }

    /**
     * Removes an item from the list of items in the area.
     * @param item The ItemInfo object to remove.
     * @return true if the item was removed, false otherwise.
     */
    public boolean removeItem(ItemInfo item) {
        if (this.items != null) {
            return this.items.remove(item);
        }
        return false;
    }
}

class ItemInfo {
    // Example fields for landmarks or treasures
    private String name;
    private int number; // Count of the item

    public ItemInfo(String name, int number) {
        this.name = name;
        this.number = number;
    }

    // Getters and setters
    public String getName() { return name; }
    public int getNumber() { return number; }

    @Override
    public String toString() {
        return "ItemInfo{name='" + name + "', number=" + number + " }";
    }
}

class ArucoResult {
    public Mat corners;
    public double id;

    public ArucoResult(Mat corners, double id) {
        this.corners = corners;
        this.id = id;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("ArucoResult{");
        sb.append("id=").append((int) id);

        if (corners != null && !corners.empty()) {
            sb.append(", corners=[");
            DecimalFormat df = new DecimalFormat("#.##");

            for (int i = 0; i < corners.cols(); i++) {
                double[] xy = corners.get(0, i);
                if (xy != null && xy.length >= 2) {
                    sb.append("(").append(df.format(xy[0])).append(", ").append(df.format(xy[1])).append(")");
                } else {
                    sb.append("invalid_corner");
                }
                if (i < corners.cols() - 1) {
                    sb.append(", ");
                }
            }
            sb.append("]");
        } else {
            sb.append(", corners=null");
        }
        sb.append("}");
        return sb.toString();
    }
}