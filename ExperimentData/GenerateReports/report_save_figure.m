%% ==================== 函数定义 ====================
function report_save_figure(output_folder, format, resolution)
    % REPORT_SAVE_FIGURE 将所有生成的图形保存为指定格式
    %
    % 参数:
    %   output_folder  - 输出文件夹名称 (默认为 'ResultFigures')
    %   format         - 图片格式 (默认为 'eps')
    %   resolution     - 分辨率DPI (默认为 300)
    %
    % 支持的格式:
    %   'eps'  - Encapsulated PostScript (矢量图，推荐用于论文)
    %   'pdf'  - Portable Document Format (矢量图)
    %   'png'  - Portable Network Graphics (位图)
    %   'jpg'  - JPEG (位图，有损压缩)
    %   'tiff' - Tagged Image File Format (位图)
    %   'svg'  - Scalable Vector Graphics (矢量图)
    %
    % 说明:
    %   此函数会自动创建输出文件夹（如果不存在），并将所有图形保存为
    %   指定格式，文件名包含实验名称和图形编号

    % 设置默认参数
    if nargin < 1
        output_folder = 'ResultFigures';
    end
    if nargin < 2
        format = 'eps';
    end
    if nargin < 3
        resolution = 300;
    end

    % 验证格式
    supported_formats = {'eps', 'pdf', 'png', 'jpg', 'jpeg', 'tiff', 'tif', 'svg'};
    format = lower(format);

    if ~ismember(format, supported_formats)
        error('不支持的格式: %s。支持的格式: %s', format, strjoin(supported_formats, ', '));
    end

    % 获取print命令格式参数
    format_map = containers.Map(...
        {'eps', 'pdf', 'png', 'jpg', 'jpeg', 'tiff', 'tif', 'svg'}, ...
        {'-depsc', '-dpdf', '-dpng', '-djpeg', '-djpeg', '-dtiff', '-dtiff', '-dsvg'});

    print_format = format_map(format);

    % 创建输出文件夹（如果不存在）
    if ~exist(output_folder, 'dir')
        mkdir(output_folder);
        fprintf('已创建输出文件夹: %s\n', output_folder);
    end

    fprintf('\n开始保存图形到: %s\n', output_folder);
    fprintf('格式: %s, 分辨率: %d DPI\n', upper(format), resolution);
    fprintf('----------------------------------------\n');

    % 获取所有打开的图形窗口
    fig_handles = findall(0, 'Type', 'figure');

    % 保存每个图形窗口
    for fig_idx = 1:length(fig_handles)
        fig_handle = fig_handles(fig_idx);

        % 确定图形类型和编号
        fig_num = fig_handle.Number;
        fig_name = get(fig_handle, 'Name');

        % 生成文件名
        if isempty(fig_name)
            filename = sprintf('figure_%d.%s', fig_num, format);
        else
            % 清理文件名中的非法字符
            safe_name = matlab.lang.makeValidName(fig_name);
            filename = sprintf('%s_%d.%s', safe_name, fig_num, format);
        end

        % 构建完整路径
        filepath = fullfile(output_folder, filename);

        % 保存为指定格式
        try
            set(fig_handle, 'PaperPositionMode', 'auto');

            % 对于矢量格式（EPS, PDF, SVG），不使用分辨率参数
            if ismember(format, {'eps', 'pdf', 'svg'})
                print(fig_handle, filepath, print_format);
            else
                % 对于位图格式，使用分辨率参数
                print(fig_handle, filepath, print_format, ['-r' num2str(resolution)]);
            end

            fprintf('✓ 已保存: %s\n', filename);
        catch ME
            fprintf('✗ 保存失败 [Figure %d]: %s\n', fig_num, ME.message);
        end
    end

    fprintf('----------------------------------------\n');
    fprintf('图形保存完成！共保存 %d 个图形。\n', length(fig_handles));
    fprintf('输出文件夹: %s\n', output_folder);
end