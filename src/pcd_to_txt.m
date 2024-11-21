% 读取 PCD 文件
pcdFile = '/home/polarbear/data/DeltaElect/mapping/fast_lio_ros2/synced_data/MD6F_Orientations_1/loop5/sparse/0/test.pcd'; % 请将此处替换为您的 PCD 文件名
ptCloud = pcread(pcdFile);

% 对点云进行降采样，降低点的数量
% 使用体素栅格降采样（Voxel Grid Downsampling）
gridSize = 0.1; % 设置体素大小，值越大，点数越少，请根据需要调整
ptCloud = pcdownsample(ptCloud, 'gridAverage', gridSize);

% 提取点云坐标
X = ptCloud.Location(:,1);
Y = ptCloud.Location(:,2);
Z = ptCloud.Location(:,3);

numPoints = length(X);

% 检查是否有 intensity 信息
if isfield(ptCloud, 'Intensity')
    % 提取强度信息并归一化到 0-255，映射为 RGB 值
    intensity = ptCloud.Intensity;

    % 将 intensity 归一化到 0-255
    intensity_norm = uint8(255 * mat2gray(intensity));

    % 将强度映射到 RGB 颜色（这里简单地将 intensity 作为灰度值映射到 R、G、B）
    R = intensity_norm;
    G = intensity_norm;
    B = intensity_norm;
else
    % 如果没有 intensity，就为每个点随机生成 RGB 值（0-255 的整数）
    R = randi([0, 255], numPoints, 1, 'uint8');
    G = randi([0, 255], numPoints, 1, 'uint8');
    B = randi([0, 255], numPoints, 1, 'uint8');
end

% 生成随机的 ERROR（0 到 1 的小数）和 TRACK（0 到 100 的整数）
ERROR = rand(numPoints, 1);
TRACK = randi([0, 100], numPoints, 1);

% 生成 POINT3D_ID
POINT3D_ID = (0:numPoints-1)';

% 计算平均 track 长度
mean_track_length = mean(TRACK);

% 准备写入文件的数据
data = [POINT3D_ID, X, Y, Z, double(R), double(G), double(B), ERROR, TRACK];

% 打开文件写入
txtFile = 'points3D.txt'; % 输出文件名
fileID = fopen(txtFile, 'w');

% 写入前三行
fprintf(fileID, '# 3D point list with one line of data per point:\n');
fprintf(fileID, '#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n');
fprintf(fileID, '# Number of points: %d, mean track length: %f\n', numPoints, mean_track_length);

% 写入数据
formatSpec = '%d %f %f %f %d %d %d %f %d\n';
fprintf(fileID, formatSpec, data');

% 关闭文件
fclose(fileID);

disp('数据已成功写入到 output.txt 文件中。');