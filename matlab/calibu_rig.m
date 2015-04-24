classdef calibu_rig
    %CALIBU_RIG Wrapper for Calibu rig class.
    %   Allows projecting, unprojecting and transferring 2d/3d from
    %   all supported camera models.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
         cpp_calibu_rig_ptr_;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods

        %%% Constructor.
        function obj = calibu_rig(filename)
            obj.cpp_calibu_rig_ptr_ = calibu_mex('new', filename);
        end

        %%% Destructor.
        function delete(obj)
            calibu_mex('delete', obj.cpp_calibu_rig_ptr_);
        end
        
        %%% Project.
        function [pixel_coordinate] = project(obj, camera_id, point)
            pixel_coordinate = calibu_mex('project', obj.cpp_calibu_rig_ptr_, ...
                       camera_id, point);
        end

        %%% Project Points.
        function [pixel_coordinate] = project_points(obj, camera_id, points)
            pixel_coordinate = calibu_mex('project_points', obj.cpp_calibu_rig_ptr_, ...
                       camera_id, length(points), points);
        end
        
        %%% Unproject.
        function [ray] = unproject(obj, camera_id, pixel_coordinate)
            ray = calibu_mex('unproject', obj.cpp_calibu_rig_ptr_, ...
                       camera_id, pixel_coordinate);
        end

        %%% Unproject Pixels.
        function [rays] = unproject_pixels(obj, camera_id, pixels)
            rays = calibu_mex('unproject_pixels', obj.cpp_calibu_rig_ptr_, ...
                       camera_id, length(pixels), pixels);
        end
        
        %%% Transfer3d.
        function [pixel_coordinate] = transfer_3d(obj, camera_id, Tab, ray, rho)
            pixel_coordinate = calibu_mex('transfer_3d', ...
                                          obj.cpp_calibu_rig_ptr_, ...
                                          camera_id, Tab, ray, rho);
        end

        %%% Get K.
        function [K] = get_K(obj, camera_id)
            K = calibu_mex('get_K', obj.cpp_calibu_rig_ptr_, camera_id);
        end

        %%% Get Trc.
        function [Trc] = get_Trc(obj, camera_id)
            Trc = calibu_mex('get_Trc', obj.cpp_calibu_rig_ptr_, camera_id);
        end
        
    end
    
end

