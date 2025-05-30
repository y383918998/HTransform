% ����ڲ� %
focalLength    = [309.4362, 344.2161]; % [fx, fy] ������Ϊ��λ
principalPoint = [318.9034, 257.5352]; % [cx, cy] �������ĵĹ�ѧ����?
imageSize      = [360, 640];           % [nrows, mcols]ͼ���С
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
% ����ⲿλ����Ϣ %
height = 2.1798;    % �����İ�װ�߶ȣ�����Ϊ��λ��?
pitch  = 14;        % ������ĸ����Ƕȣ��Զ�Ϊ��λ��?
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);% ����һ����Ŀ���?
[file, path] = uigetfile({'*.mp4;*.avi', 'Video Files (*.mp4, *.avi)'}, 'Select a Video File');
if isequal(file, 0)
    disp('User selected Cancel');
else
    disp(['User selected ', fullfile(path, file)]);
    % Ȼ��ʹ��VideoReader��ȡ��Ƶ��
    videoReader = VideoReader(fullfile(path, file));
end
% ������Ȥ�Ĳ��֣����а���������־�ͳ���??
timeStamp = 0;                   % ����Ƶ��ʼ��ʱ�� 
videoReader.CurrentTime = timeStamp;   % ָ����ѡ֡
frame = readFrame(videoReader);        % ��timeStamp���ȡ֡
% �ڳ�������ϵ��ת��Ϊ���ͼ������Ϊǰ��3-13�ף�����6�� %
distAheadOfSensor = 13; 
spaceToOneSide    = 6;  
bottomOffset      = 3;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250];
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);


detector = vehicleDetectorACF();
vehicleWidth = [1.5, 2.5];            % ��ͨ�����Ŀ����1.5��2.5��֮��?
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);    % ����AFC̽����������ͷ?
[bboxes, scores] = detect(monoDetector, frame);
locations = computeVehicleLocations(bboxes, sensor);
%  imgOut = insertVehicleDetections(frame, locations, bboxes); % ���������������Ƶ֡��

videoReader.CurrentTime = 0;
isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
 
    % ץȡ��Ƶ֡?
    frame = readFrame(videoReader);
    % ���ͼ?
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);
    % ��⳵���߽�����?
    approxLaneMarkerWidthVehicle = 0.25;    % �������25cm
    vehicleROI = outView - [-1, 2, -3, 3];  % [4,11,-3,3]
    laneSensitivity = 0.25;                 % ������?
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig,approxLaneMarkerWidthVehicle, 'ROI', vehicleROI,'Sensitivity', laneSensitivity);         %输入灰度图转化为二�?�图
    % ����������ϵ�³����ߵ�ת������������ϵ��
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
    % Ѱ�ҳ����߽��ѡ��?
    maxLanes      = 2;                      % Ѱ�������������?
    boundaryWidth = 3*approxLaneMarkerWidthVehicle; % ��չ�߽�����������������?
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);   % ���������߳����߽�?
    % 根据长度 筛�??
    maxPossibleXLength = diff(vehicleROI(1:2));      % ѡȡ����Ȥ����ROI�У�x�������󳤶�?
    minXLength         = maxPossibleXLength * 0.6;   % ������С�����ż�?
    isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
    boundaries    = boundaries(isOfMinLength);
    % ����ǿ�ȣ�ɸѡ?%
    birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
    [laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
    vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);% ��ͼ���ת��Ϊ������
    maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));%unique��vehiclePoints�е�һ��Ԫ�أ� �����κγ������ܵ����ΨһX��λ����
    maxLaneLength = diff(vehicleROI(1:2));                 % �������߽����󳤶�����ΪROI���ȣ�28-4=24
    maxStrength   = maxPointsInOneLane/maxLaneLength;      % �����֡������ߴ�/ ROI�ߴ�=��󳵵�ǿ�� ?  
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);
    boundaries = classifyLaneTypes(boundaries, boundaryPoints);% ���೵���������
    % Ѱ������ͨ��
    xOffset    = 0;   % ���봫����0��?
    distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
    % Ѱ�Һ�ѡ���ұ߽�?
    leftEgoBoundaryIndex  = [];
    rightEgoBoundaryIndex = [];
    minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
    minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
    if ~isempty(minLDistance)
        leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
    end
    if ~isempty(minRDistance)
        rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
    end
    leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
    rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);
    % ��⳵��?
    [bboxes, scores] = detect(monoDetector, frame);
    locations = computeVehicleLocations(bboxes, sensor);
    % ���������?
    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;
    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;
    % ����������ӻ����ݣ������м���
    intOut.birdsEyeImage   = birdsEyeImage;
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;
    closePlayers = ~hasFrame(videoReader);
    isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
        intOut, closePlayers);
    timeStamp = 2; % ��2s��ʼ��ʾ?
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end
end
