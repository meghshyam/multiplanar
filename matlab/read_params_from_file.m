function [plane_bounding_box_points, plane_parameters] = read_params_from_file(file_name, debug)

    if nargin == 0
        file_name = 'Plane_Info.txt';
        debug = 1;
    elseif nargin == 1
        debug = 1;
    end
    fileID = fopen(file_name,'r');
    line_no = 1;
    plane_line_no = 1;
    plane_bounding_box_points = [];
    plane_parameters = [];
    while ~feof(fileID)
        fprintf(1, 'Line no: %d, Plane line no: %d\n', line_no, plane_line_no)
        if plane_line_no == 6
            plane_line_no = 1;
            line = fgets(fileID);
            % fprintf(1, line);
            A = sscanf(line,'%s');
            line = fgets(fileID);
            % fprintf(1, line);
            A = cell2mat(textscan(line,'%f'));
            plane_parameters = [plane_parameters; A'];
            line = fgets(fileID);
            line_no = 0;
        elseif line_no ~= 1 && line_no ~= 2
            line = fgets(fileID);
            % fprintf(1, line);
            A = cell2mat(textscan(line,'%f'));
            plane_bounding_box_points = [plane_bounding_box_points; A'];
            plane_line_no = plane_line_no + 1;
        else
            line = fgets(fileID);
            fprintf(1, line);
        end
        line_no  = line_no + 1;
    end
    fclose(fileID);
    if debug == 1
        plane_bounding_box_points
        plane_parameters
    end
end