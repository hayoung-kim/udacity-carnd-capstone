### Traffic light classifier

Since we know the locations of the traffic lights and the vechile, we can get reduce the classification problem to transformation and detection problem. Color is easier to detect in HSV space.  In our use case, red light is very important and in HSV space red has two different ranges, since we want to be very sensitive to red light, I include both range in the the mask.  Further improments can be made when dealing with unknow locations and complex data by applying Deep NN solutions. 

```python

        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # red has hue 0 - 10 & 160 - 180 add another filter 
        # TODO  use Guassian mask
        RED_MIN1 = np.array([0, 100, 100],np.uint8)
        RED_MAX1 = np.array([10, 255, 255],np.uint8)        

        RED_MIN2 = np.array([160, 100, 100],np.uint8)
        RED_MAX2 = np.array([179, 255, 255],np.uint8)

        frame_threshed1 = cv2.inRange(hsv_img, RED_MIN1, RED_MAX1) 
        frame_threshed2 = cv2.inRange(hsv_img, RED_MIN2, RED_MAX2) 
        if cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2) > 50:
            return TrafficLight.RED

        YELLOW_MIN = np.array([40.0/360*255, 100, 100],np.uint8)
        YELLOW_MAX = np.array([66.0/360*255, 255, 255],np.uint8)
        frame_threshed3 = cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)
        if cv2.countNonZero(frame_threshed3) > 50:
            return TrafficLight.YELLOW

        GREEN_MIN = np.array([90.0/360*255, 100, 100],np.uint8)
        GREEN_MAX = np.array([140.0/360*255, 255, 255],np.uint8)
        frame_threshed4 = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)
        if cv2.countNonZero(frame_threshed4) > 50:
            return TrafficLight.GREEN

```


### Traffic light detection 


For each stop line positions, we'll add the nearest waypoints and index (defined by the shortest Euclidean distance ) and then if we found a light from the camera,we'll use it's location for transformation


```python
        idx = 0
        for line in stop_line_positions:
            traffic_light = TrafficLight()
            traffic_light.pose = PoseStamped()
            traffic_light.pose.pose.position.x = line[0]
            traffic_light.pose.pose.position.y = line[1]
            traffic_light.pose.pose.position.z = 0.0
            self.traffic_light_waypoint_indexes.append([idx,self.get_closest_waypoint(traffic_light.pose.pose)])
            idx += 1
```

Image first project to image plane and then cropped to target place 

```python
        x, y = self.project_to_image_plane(light.pose.pose.position)

        if (x<0) or (y<0) or (x>=cv_image.shape[1]) or (y>=cv_image.shape[0]):
            return False


        imm = cv_image
        crop = 90
        xmin = x - crop if (x-crop) >= 0 else 0
        ymin = y - crop if (y-crop) >= 0 else 0

        xmax = x + crop if (x + crop) <= imm.shape[1]-1 else imm.shape[1]-1
        ymax = y + crop if (y + crop) <= imm.shape[0]-1 else imm.shape[0]-1
        imm_cropped = imm[ymin:ymax,xmin:xmax]

```


