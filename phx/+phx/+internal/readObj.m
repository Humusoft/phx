function OBJ = readObj(objFileName)
% Read the objects from a Wavefront OBJ file
%
% OBJ = readObj(filename);
%
% OBJ struct containing:
%
% OBJ.vertices: Vertices coordinates
% OBJ.vertices_texture: Texture coordinates
% OBJ.vertices_normal: Normal vectors
% OBJ.vertices_point: Vertex data used for points and lines
% OBJ.material: Parameters from external .MTL file, will contain parameters like
%               newmtl, Ka, Kd, Ks, illum, Ns, map_Ka, map_Kd, map_Ks,
%       example of an entry from the material object:
%       OBJ.material(i).type = newmtl
%       OBJ.material(i).data = 'vase_tex'
% OBJ.objects: Cell object with all objects in the OBJ file,
%       example of a mesh object:
%       OBJ.objects(i).type = 'f'
%       OBJ.objects(i).data.vertices: [n x 3 double]
%       OBJ.objects(i).data.texture:  [n x 3 double]
%       OBJ.objects(i).data.normal:   [n x 3 double]
%
% Function is written by D.Kroon University of Twente (June 2010)
%
% Reworked, fixed and optimized by Lubor Zhanal (2026)

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    % Read the textfile
    fid = fopen(objFileName, "r");
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = lines{1};
    nLines = numel(lines);
    
    % Preallocate arrays
    vertices = zeros(nLines, 3);
    nv = 0;
    vertices_texture = zeros(nLines, 2);
    nvt = 0;
    vertices_normal = zeros(nLines, 3);
    nvn = 0;
    material = [];
    objects(nLines).type = [];
    objects(nLines).data = [];
    no = 0;
    
    % Loop through the Wavefront object file
    for iln = 1:nLines
        % Split line to type and data
        line = lines{iln};
        id = find(line == ' ', 1);
        if ~isempty(id)
            type = line(1:id - 1);
            data = strtrim(line(id:end));
        else
            continue
        end
    
        % Switch on data type line
        switch type
            case 'mtllib'
                filefolder = fileparts(objFileName);
                filename_mtl = fullfile(filefolder, data);
                material = readmtl(filename_mtl);
            case 'v'
                % Vertices
                nv = nv + 1;
                vertices(nv, :) = sscanf(data, '%f')';
            case 'vp'
                % Specifies a point in the parameter space of curve or surface
                % skip
            case 'vn'
                % A normal vector
                nvn = nvn + 1;
                vertices_normal(nvn, :) = sscanf(data, '%f')';
            case 'vt'
                % Vertices Texture Coordinate in photo
                % U V W
                nvt = nvt + 1;
                uvw = sscanf(data, '%f')';
                vertices_texture(nvt, :) = [uvw(1) 1-uvw(2)];
            case 'l'
                % skip lines
            case 'f'
                no = no + 1;
                data = regexp(data, '\s+', 'split');
                nData = numel(data);
                array_vertices = zeros(1, nData);
                array_texture = zeros(1, nData);
                array_normal = zeros(1, nData);
                for i = 1:nData
                    tstr = strrep(data{i}, '//', '/NaN/');
                    tstr = strrep(tstr, '/', ' ');
                    tvals = sscanf(tstr, '%f')';
    
                    val = tvals(1);
                    if val < 0
                        val = val + 1 + nv;
                    end
                    
                    array_vertices(i) = val;
                    if length(tvals) > 1
                        if isfinite(tvals(2))
                            val = tvals(2);
                            if val < 0
                                val = val + 1 + nvt;
                            end
                            array_texture(i) = val;
                        end
                    end
    
                    if length(tvals) > 2
                        val = tvals(3);
                        if val < 0
                            val = val + 1 + nvn;
                        end
                        array_normal(i) = val;
                    end
                end
    
                % A face of more than 3 indices is always split into
                % multiple faces of only 3 indices.
                objects(no).type = 'f';
                findex = 1:min(3, length(array_vertices));
    
                objects(no).data.vertices = array_vertices(findex);
                if ~isempty(array_texture)
                    objects(no).data.texture = array_texture(findex);
                end
                if ~isempty(array_normal)
                    objects(no).data.normal = array_normal(findex);
                end
                for i = 1:length(array_vertices)-3
                    no = no + 1;
                    findex = [1 2+i 3+i];
                    findex(findex>length(array_vertices)) = findex(findex > length(array_vertices)) - length(array_vertices);
                    objects(no).type = 'f';
                    objects(no).data.vertices = array_vertices(findex);
                    if ~isempty(array_texture)
                        objects(no).data.texture = array_texture(findex);
                    end
                    if ~isempty(array_normal)
                        objects(no).data.normal = array_normal(findex);
                    end
                end
            case {'#', '$'}
                % skip comments
            case ''
                % skip empty lines
            otherwise
                no = no + 1;
                objects(no).type = type;
                objects(no).data = data;
        end
    end

    % Initialize new object list, which will contain the "collapsed" objects
    objects2(no).data = 0;
    index = 0;
    i = 0;
    while i < no
        i = i + 1;
        type = objects(i).type;
        % First face found
        if strcmp(type, 'f')
            % Get number of faces
            for j = i:no
                type = objects(j).type;
                if (length(type) ~= 1) || (type(1) ~= 'f')
                    j = j - 1;
                    break
                end
            end
            numfaces = j - i + 1;
    
            index = index + 1;
            objects2(index).type = 'f';
            % Process last face first to allocate memory
            objects2(index).data.vertices(numfaces, :) = objects(i).data.vertices;
            if isfield(objects(i).data,'texture')
                objects2(index).data.texture(numfaces, :) = objects(i).data.texture;
            else
                objects2(index).data.texture = [];
            end
            if isfield(objects(i).data, 'normal')
                objects2(index).data.normal(numfaces, :) = objects(i).data.normal;
            else
                objects2(index).data.normal = [];
            end
            % All faces to arrays
            for k = 1:numfaces
                objects2(index).data.vertices(k,:) = objects(i+k-1).data.vertices;
                if isfield(objects(i).data, 'texture')
                    objects2(index).data.texture(k, :) = objects(i+k-1).data.texture;
                end
                if isfield(objects(i).data, 'normal')
                    objects2(index).data.normal(k, :) = objects(i+k-1).data.normal;
                end
            end
            i = j;
        else
            index=index+1;
            objects2(index).type=objects(i).type;
            objects2(index).data=objects(i).data;
        end
    end

    % Add all data to output struct
    OBJ.objects = objects2(1:index);
    OBJ.material = material;
    OBJ.vertices = vertices(1:nv, :);
    OBJ.vertices_normal = vertices_normal(1:nvn, :);
    OBJ.vertices_texture = vertices_texture(1:nvt, :);
end

function  objects = readmtl(mtlFileName)
    % Read the textfile
    fid = fopen(mtlFileName, "r");
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = lines{1};
    nLines = numel(lines);

    % Preallocate arrays
    objects(nLines).type = [];
    objects(nLines).data = [];
    no = 0;
    
    % Loop through the Wavefront object file
    for iln = 1:nLines
        % Split line to type and data
        line = lines{iln};
        id = find(line == ' ', 1);
        if ~isempty(id)
            type = line(1:id - 1);
            data = strtrim(line(id:end));
        else
            continue
        end
    
        % Switch on data type line
        switch type
            case {'#','$'}
                % skip comment
            case {''}
                % skip empty line
            otherwise
                no = no + 1;
                objects(no).type = type;
                objects(no).data = data;
        end
    end
    objects = objects(1:no);
end