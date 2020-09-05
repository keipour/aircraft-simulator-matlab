classdef (Abstract) collision_base < handle
%This class is for internal use only. It may be removed in the future.

%CollisionGeometryBase Base class for collision geometries

%   Copyright 2019 The MathWorks, Inc.

    properties (Dependent)
        %Pose Pose of the geometry relative to the world frame
        %
        %   Default: eye(4)
        Pose
        
    end
    
    properties (Access = protected)
        
        %PoseInternal
        PoseInternal

        %VisualMeshVertices
        VisualMeshVertices
        
        %VisualMeshFaces
        VisualMeshFaces
        
        %EstimatedMaxReach
        EstimatedMaxReach
    end
    
    properties (Hidden, SetAccess = protected)
        %Position
        Position
        
        %Quaternion
        Quaternion
    end
    
    properties (Hidden, Transient, SetAccess = protected)
                
        %GeometryInternal Built-in representation of the collision geometry
        GeometryInternal
                
    end

    methods

        function obj = collision_base()
            %CollisionGeometryBase
            obj.Pose = eye(4);

        end
        
        function [axOut, patchObjOut] = show(obj, varargin)
            %SHOW Plot collision geometry
            %   SHOW(GEOM) plots GEOM in MATLAB figure at its current pose. 
            %   The tessellation will be generated automatically for
            %   primitives.
            %   
            %   AX = SHOW(GEOM) returns the axes handle under which
            %   the collision geometry GEOM is plotted.
            %
            %   [AX, PATCHOBJ] = show(GEOM, ___) returns as a second output  
            %   argument the graphic object (patch) that represents 
            %   the collision geometry in AX.
            %
            %   SHOW(___, Name, Value) provides additional options specified
            %   by one or more Name, Value pair arguments. Name must appear
            %   inside single quotes (''). You can specify several name-value
            %   pair arguments in any order as Name1, Value1, ..., NameN, ValueN:
            %
            %      'Parent'         - Handle of the axes in which the
            %                         collision geometry is to be rendered
            
            % parse show inputs
            parser = inputParser;
            parser.StructExpand = false;
            parser.addParameter('Parent', [], ...
                @(x)robotics.internal.validation.validateAxesHandle(x));
            
            parser.parse(varargin{:});
            
            ph = parser.Results.Parent;

            if isempty(ph)
                ax = newplot; 
            else
                ax = newplot(ph);
            end
            
            if strcmp(ax.NextPlot, 'replace') % when hold is off, reset scene
                resetScene(obj, ax);
            end
            
            q = quaternion(obj.Quaternion);
            
            Vt = obj.Position + q.rotatepoint(obj.VisualMeshVertices);
            
            if isempty(obj.VisualMeshFaces)
                patchObj = patch(ax, 'Vertices', Vt, ...
                  'FaceColor', [1, 0.5, 0]);
            else
                patchObj = patch(ax,'Faces', obj.VisualMeshFaces,...
                  'Vertices', Vt, ...
                  'FaceColor', [1, 0.5, 0]);                
            end
            
            % only return outputs if user requests it
            if nargout > 0
                axOut = ax;
            end
            
            if nargout > 1
                patchObjOut = patchObj;
            end
        end

    end
    
    methods
        function set.Pose(obj, tform)
            %set.Pose
            obj.updatePose(tform);
        end
        
        function pose = get.Pose(obj)
            %get.Pose
            pose = obj.PoseInternal;
        end
    end
    
    methods (Access = protected)
        function resetScene(obj, ax)
            %resetScene Reset scene if hold is off
            
            axis(ax, 'vis3d');
            hFigure = ax.Parent;
            pan(hFigure,'on');
            rotate3d(hFigure,'on');
            
            ax.Visible = 'off';
            daspect(ax, [1 1 1]);
            
            % center the geometry 
            a = obj.EstimatedMaxReach;
            if obj.EstimatedMaxReach == 0.0
                a = 1e-10;
            end
            set(ax, 'xlim', obj.Position(1) + [-a, a], 'ylim', obj.Position(2) + [-a, a], 'zlim', obj.Position(3) + [-a, a]);
            
            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');
            
            % set up view 
            view(ax, [135 8]);
            set(ax, 'Projection', 'perspective');
            ax.CameraViewAngle = 8.0;
            grid(ax, 'on');
            
            % set up lighting
            if isempty(findobj(ax,'Type','Light'))
                light('Position',[a, 0, a],'Style','local','parent',ax);
            end
            ax.Visible = 'on';
            
        end
        
        function updatePose(obj, pose)
            %updatePose

            pos = pose(1:3, 4)';
            quat = physics.RotationMatrixToQuaternion(pose(1:3, 1:3));
            obj.Position = pos;         
            obj.Quaternion = quat;
            
            obj.PoseInternal = pose;

        end
    end
end
