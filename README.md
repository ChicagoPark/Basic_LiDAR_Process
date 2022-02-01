# LiDAR_Processing
Lidar, Pointcloud, perception

## 0. Sensor Arrangement

<img width="600" alt="LiDAR_Arrange" src="https://user-images.githubusercontent.com/73331241/139469477-5e33ac45-71a2-47df-b833-db787c210d53.jpg">


## 1. Multi LiDAR Merging

<img width="600" alt="download (1)" src="https://user-images.githubusercontent.com/73331241/139469578-b9f4ce1e-fa20-4186-8faf-04d37d0fcd44.png">
```python
path: my_pcl_tutorial/src/merging_ALL.cpp
```

## 2. Segmentation : Extracting Obstacle or Lane Plane
### `Getting Obstacle PointCloud ONLY`

<img width="700" alt="Untitled (1)" src="https://user-images.githubusercontent.com/73331241/139468697-7cd214bb-7560-4e51-aced-240241c0c458.png">

### `Getting Lane Plane PointCloud ONLY`

<img width="700" alt="Untitled" src="https://user-images.githubusercontent.com/73331241/139468550-14f0a159-4de8-42e2-9741-c5dd17b175a2.png">
```python
path: my_pcl_tutorial/src/segmentation.cpp
```

## 3. Clustering

<img width="700" alt="download" src="https://user-images.githubusercontent.com/73331241/139470434-223cd671-94b5-4f6b-ad77-78ad83d8e9d1.png">
```python
path: my_pcl_tutorial/src/clustering.cpp
```

## 4. Bounding Box generating

<img width="700" alt="download (2)" src="https://user-images.githubusercontent.com/73331241/139470442-0405e22c-2cdf-46de-a235-fd1498902236.png">

```python
path: my_pcl_tutorial/src/boundingbox.cpp
```
