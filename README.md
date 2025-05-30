```markdown
# HTransform 项目说明

## 项目概述
本项目主要实现了对视频中车辆和车道线的检测与可视化功能。它通过单目相机模型，结合车辆检测算法和车道线分割算法，将视频中的车辆位置和车道线信息提取出来，并以鸟瞰图和常规视图两种方式进行可视化展示。

## 功能模块
### 1. 相机参数设置
在 `main.m` 文件中，首先设置了相机的内参和外参，包括焦距、主点、图像尺寸、安装高度和俯仰角等。这些参数用于创建相机模型，为后续的图像转换和目标检测提供基础。
```matlab
focalLength    = [309.4362, 344.2161]; % [fx, fy] 焦距，单位为像素
principalPoint = [318.9034, 257.5352]; % [cx, cy] 主点的几何坐标
imageSize      = [360, 640];           % [nrows, mcols] 图像尺寸
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
height = 2.1798;    % 相机的安装高度，单位为米
pitch  = 14;        % 相机的俯仰角度，单位为度
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch); % 创建单目相机模型
```
### 2. 视频读取
使用 `uigetfile` 函数让用户选择要处理的视频文件，然后通过 `VideoReader` 读取视频帧。
```matlab
[file, path] = uigetfile({'*.mp4;*.avi', 'Video Files (*.mp4, *.avi)'}, 'Select a Video File');
if isequal(file, 0)
    disp('User selected Cancel');
else
    disp(['User selected ', fullfile(path, file)]);
    videoReader = VideoReader(fullfile(path, file));
end
```
### 3. 鸟瞰图转换
通过 `birdsEyeView` 函数创建鸟瞰图配置对象，将相机视角的图像转换为鸟瞰图。
```matlab
distAheadOfSensor = 13; 
spaceToOneSide    = 6;  
bottomOffset      = 3;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250];
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
birdsEyeImage = transformImage(birdsEyeConfig, frame);
```
### 4. 车辆检测
使用 `vehicleDetectorACF` 检测器进行车辆检测，并通过 `computeVehicleLocations` 函数计算车辆在实际世界中的位置。
```matlab
detector = vehicleDetectorACF();
vehicleWidth = [1.5, 2.5];            % 普通车辆的宽度在1.5到2.5米之间
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);    % 配置AFC车辆检测器和单目相机
[bboxes, scores] = detect(monoDetector, frame);
locations = computeVehicleLocations(bboxes, sensor);
```
### 5. 车道线分割与检测
将鸟瞰图转换为灰度图后，使用 `segmentLaneMarkerRidge` 函数进行车道线分割，然后通过 `findParabolicLaneBoundaries` 函数找到车道线边界，并进行筛选和分类。
```matlab
birdsEyeImage = rgb2gray(birdsEyeImage);
approxLaneMarkerWidthVehicle = 0.25;    % 车道线标记的近似宽度为25cm
vehicleROI = outView - [-1, 2, -3, 3];  % [4,11,-3,3]
laneSensitivity = 0.25;                 % 分割灵敏度
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig,approxLaneMarkerWidthVehicle, 'ROI', vehicleROI,'Sensitivity', laneSensitivity);
[imageX, imageY] = find(birdsEyeViewBW);
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
maxLanes      = 2;                      % 最多检测2条车道线
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % 扩展边界宽度
[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
    'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);
```
### 6. 可视化展示
使用 `visualizeSensorResults` 函数将检测到的车辆和车道线信息可视化，包括常规视图和鸟瞰图。
```matlab
isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
    intOut, closePlayers);
```

## 文件说明
- `main.m`：主程序文件，负责相机参数设置、视频读取、车辆和车道线检测以及可视化展示。
- `computeVehicleLocations.m`：计算车辆在实际世界中的位置。
- `takeSnapshot.m`：拍摄包含车辆和车道线检测结果的快照。
- `validateBoundaryFcn.m`：验证车道线边界的有效性。
- `classifyLaneTypes.m`：对车道线类型进行分类（实线或虚线）。
- `insertVehicleDetections.m`：在图像中插入车辆检测结果。
- `visualizeSensorResults.m`：可视化传感器检测结果。
- `vehicleToImageROI.m`：将车辆坐标系下的ROI转换为图像坐标系下的ROI。
- `helperVideoPlayerSet.m`：辅助类，用于管理多个视频显示窗口。

## 使用方法
1. 运行 `main.m` 文件。
2. 在弹出的文件选择对话框中选择要处理的视频文件。
3. 程序将自动进行车辆和车道线检测，并在窗口中显示检测结果。

## 注意事项
- 确保安装了MATLAB的Computer Vision Toolbox和Image Processing Toolbox，因为部分函数依赖这些工具箱。
- 视频文件格式必须为 `.mp4` 或 `.avi`。

```
