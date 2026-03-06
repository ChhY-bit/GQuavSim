%% 导入ExperimentData文件夹内所有.mat文件
% 此脚本用于加载当前文件夹内的所有.mat文件并进行基本分析

function all_data = report_read_exprdata()
    fprintf('----------------------------------------\n');
    % 获取当前文件夹内所有.mat文件
    mat_files = dir('*.mat');

    % 检查是否有.mat文件
    if isempty(mat_files)
        fprintf('当前文件夹内没有.mat文件！\n');
        all_data = [];
        return;
    end
    fprintf('找到 %d 个.mat文件：\n', length(mat_files));
    fprintf('----------------------------------------\n');

    % 初始化数据存储结构
    all_data = cell(length(mat_files),1);
    file_names = cell(length(mat_files),1);

    % 遍历所有.mat文件
    for i = 1:length(mat_files)
        filename = mat_files(i).name;
        fprintf('[%d/%d]\t正在加载: %s\n', i, length(mat_files), filename);
        try
            % 加载.mat文件
            loaded_data = load(filename);
            % 存储数据
            all_data{i} = loaded_data;
            file_names{i} = filename;
            fprintf('\t✓ 加载成功\n');
        catch ME
            fprintf('\t✗ 加载失败: %s\n', ME.message);
        end
    end
    fprintf('----------------------------------------\n');
    fprintf('所有.mat文件加载完成！\n');
    fprintf('已加载的文件: %d 个\n', length(all_data));
    fprintf('----------------------------------------\n');

end


