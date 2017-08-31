classdef AvoidObstacles < simiam.controller.Controller

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software

    properties
        
        % memory banks
        E_k
        e_k_1
        
        % gains
        Kp
        Ki
        Kd
        
        % plot support
        p
        
        % IR plot support
        u_arrow
        u_arrow_r
        s_net
        
        % sensor geometry
        calibrated
        sensor_placement
    end
    
    properties (Constant)
        inputs = struct('v', 0);
        outputs = struct('v', 0, 'w', 0)
    end
    
    methods
        
        function obj = AvoidObstacles()
            obj = obj@simiam.controller.Controller('avoid_obstacles');            
            obj.calibrated = false;
            
            obj.Kp = 5;
            obj.Ki = 0.0001;
            obj.Kd = 1; 
            
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            
            % Compute the placement of the sensors
            if(~obj.calibrated)
                obj.set_sensor_geometry(robot);
                
                % Temporoary plot support
                hold(robot.parent, 'on');
                obj.u_arrow_r = plot(robot.parent, [0 0], [0 0], 'b-x', 'LineWidth', 2);
                obj.u_arrow = plot(robot.parent, [0 0], [0 0], 'r--x', 'LineWidth', 2);
                obj.s_net = plot(robot.parent, zeros(1,5), zeros(1,5), 'kx', 'MarkerSize', 8);
                set(obj.u_arrow_r, 'ZData', ones(1,2));
                set(obj.u_arrow, 'ZData', ones(1,2));
                set(obj.s_net, 'ZData', ones(1,5));
            end
            
            % Unpack state estimate
            [x, y, theta] = state_estimate.unpack();
            
            % Poll the current IR sensor values 1-5
            ir_distances = robot.get_ir_distances();           
            
                        
            % Interpret the IR sensor measurements geometrically
            ir_distances_wf = obj.apply_sensor_geometry(ir_distances, state_estimate);            
            
            %% START CODE BLOCK %%
            
            % Compute the heading vector
            
            sensor_max = 0.3;
            n_sensors = length(robot.ir_array);
            sensor_gains = [55 55 50 50 50];
            
            u_i = ir_distances_wf(1:2, :) - [x;y]; % vectors pointing from robot to each sensor w.r.t. world frame
            u_ao = u_i * sensor_gains';
            u_a = [0; 0];
            % detect whether there is obstacle on the left, using the two
            % points in world frame that the two sensors on the left read
            u_left_gain = 0;
            u_left = [0; 0];
            if sum(ir_distances(1:2) < 0.75 * sensor_max) > 0
                ir_left = ir_distances_wf(:, 1:2);           % get the WF coordinates of the two points
                ob_left = (ir_left(:,2) - ir_left(:,1)) / norm (ir_left(:,2) - ir_left(:,1));   % get the normalized vector of the obstcle edge defined by the two points
                u_ap = [x; y] - ir_left(2);       % a vector from a point on obstacle to robot center
                u_fw_tp = u_ap - (u_ap' * ob_left) * ob_left; % normal vector pointing to robot center
 %               if norm(u_fw_tp) < 0.5 * sensor_max
                    u_left_gain = 1/norm(u_fw_tp);
 %               end
                
                u_left = u_fw_tp * u_left_gain;
                u_a = u_a + u_left
           end
%            u_left = u_left - [x; y];

            u_right_gain = 0;
            u_right = [0; 0];
            if sum(ir_distances(4:5) < 0.75 * sensor_max) > 0
                ir_right = ir_distances_wf(:, 4:5);           % get the WF coordinates of the two points
                ob_right = (ir_right(:,2) - ir_right(:,1)) / norm (ir_right(:,2) - ir_right(:,1));   % get the normalized vector of the obstcle edge defined by the two points
                u_ap = [x; y] - ir_right(1);       % a vector from a point on obstacle to robot center
                u_fw_tp = u_ap - (u_ap' * ob_right) * ob_right; % normal vector pointing to robot center
  %              if norm(u_fw_tp) < 0.5 * sensor_max
                    u_right_gain = 1/norm(u_fw_tp);
 %               end
                
                u_right = u_fw_tp * u_right_gain;
                u_a = u_a + u_right;
            end
 %           u_right = u_right - [x; y];
            
            u_front_gain = 0;
 %           u_front = u_front_gain * (ir_distances_wf(:, 3) - [x;y]) * 50;
            u_front = [0; 0];
            if sum(ir_distances(2:4) < 0.95 * sensor_max) > 0
                ir_front = ir_distances_wf(:, 2:4);           % get the WF coordinates of the two points
                [longest, idx_max] = max (ir_distances(2:4));
                [longest, idx_min] = min (ir_distances(2:4));
                u_ob = [ir_front(:,1:idx_min), ir_front(:,idx_max)];
%                 idx = setdiff ([2 3 4], idx);
%                 if ir_distances(idx(1)) > ir_distances(idx(2))
%                     u_ob = [u_ob(:,2), u_ob(:,1)];
%                 end
                ob_front = (u_ob(:,2) - u_ob(:,1)) / norm (u_ob(:,2) - u_ob(:,1));  % get the normalized vector of the obstcle edge defined by the two points
                u_ap = [x; y] - u_ob(2);       % a vector from a point on obstacle to robot center
                u_fw_tp = u_ap - (u_ap' * ob_front) * ob_front; % normal vector pointing to robot center
 %               if norm(u_fw_tp)<0.9*sensor_max
                    u_front_gain = 1/norm(u_fw_tp);
 %               end
                %            u_front = u_fw_tp * u_front_gain;
                
                u_front = ob_front * u_front_gain ;
                u_a = u_a + u_front;
            end
%            u_front = u_front - [x; y];
%            if u_a(1) ~=0 || u_a(2) ~= 0
%                u_ao = u_left + u_right + u_front;
%            end
%            u_ao = u_left;
%            sensor_gains = sensor_gains + [u_left_gain, 0, u_front_gain, 0, u_right_gain];

%             u_ao = [u_left, u_right, u_front] * [50, 50, 50]';
%             if sum(u_a) ~= 0
%                 u_ao = u_a;
%             end
           
            % Compute the heading and error for the PID controller
            % theta_ao = 0;
            theta_ao = atan2(u_ao(2) - y, u_ao(1) - x);
            % e_k = 0;
            
            % turn the range of theta from [0, 2*pi] to [-pi, pi]
%             theta1 = theta;
%             n = ceil(abs(theta)/(2*pi));
%             if theta > pi
%                 theta1 = theta - n*2*pi;
%             else
%                 if theta < -pi
%                     theta1 = theta + n*2*pi;
%                 end
%             end

            e_k = theta_ao - theta;
            e_k = atan2(sin(e_k),cos(e_k));
            
%             n = ceil(abs(e_k)/pi);
%             if e_k > pi
%                 e_k = e_k - (n+1)*pi;
%             else
%                 if e_k < -pi
%                     e_k = e_k + n*2*pi;
%                 end
%             end          
%             
%             e_k = atan2(sin(e_k),cos(e_k));
            
            %% END CODE BLOCK %%
                                    
            e_P = e_k;
            e_I = obj.E_k + e_k*dt;         % the previous accumulated error plus the current accumulated error in period dt
            e_D = (e_k-obj.e_k_1)/dt;       % the current error deducted by the previous error = error change => error change rate, i.r., e-dot
              
            % PID control on w
            v = inputs.v;
            w = obj.Kp*e_P + obj.Ki*e_I + obj.Kd*e_D;
            
            % Save errors for next time step
            obj.E_k = e_I;          % accumulated error
            obj.e_k_1 = e_k;        % the previous error
                        
            % plot
            obj.p.plot_2d_ref(dt, atan2(sin(theta),cos(theta)), theta_ao, 'g');
            
%             fprintf('(v,w) = (%0.4g,%0.4g)\n', v,w);
            
            % Temporary plot support
            u_n = u_ao/(4*norm(u_ao));
            set(obj.u_arrow, 'XData', [x x+u_n(1)]);
            set(obj.u_arrow, 'YData', [y y+u_n(2)]);
            set(obj.u_arrow_r, 'XData', [x x+0.25*cos(theta)]);
            set(obj.u_arrow_r, 'YData', [y y+0.25*sin(theta)]);
            set(obj.s_net, 'XData', ir_distances_wf(1,:));
            set(obj.s_net, 'YData', ir_distances_wf(2,:));

            % velocity control
            outputs.v = v;
            outputs.w = w;
        end
        
        % Helper functions
        
        function ir_distances_wf = apply_sensor_geometry(obj, ir_distances, state_estimate)
            n_sensors = numel(ir_distances);
            
            %% START CODE BLOCK %%
            
            % Apply the transformation to robot frame.
            
            ir_distances_rf = zeros(3,n_sensors);
            for i=1:n_sensors
                x_s = obj.sensor_placement(1,i);
                y_s = obj.sensor_placement(2,i);
                theta_s = obj.sensor_placement(3,i);
                
           %     R = obj.get_transformation_matrix(0,0,0);
                R = obj.get_transformation_matrix (x_s, y_s, theta_s);
           %     ir_distances_rf(:,i) = zeros(3,1);
                ir_distances_rf(:,i) = R * [ir_distances(i); 0; 1];
            end
            
            % Apply the transformation to world frame.
            
            [x,y,theta] = state_estimate.unpack();
            
            % R = obj.get_transformation_matrix(0,0,0);
            R = obj.get_transformation_matrix(x, y, theta);
            % ir_distances_wf = zeros(3,5);
            ir_distances_wf = R * ir_distances_rf;
            
            %% END CODE BLOCK %%
            
            ir_distances_wf = ir_distances_wf(1:2,:);
        end
        
        
        function R = get_transformation_matrix(obj, x, y, theta)
            %% START CODE BLOCK %%
%            R = zeros(3,3);
            R = [cos(theta), -sin(theta), x; 
                 sin(theta), cos(theta),  y;
                 0,          0,           1];
            
            %% END CODE BLOCK %%
        end
        
        function set_sensor_geometry(obj, robot)
            n_sensors = length(robot.ir_array);
            obj.sensor_placement = zeros(3,n_sensors);
            for i=1:n_sensors
                [x, y, theta] = robot.ir_array(i).location.unpack();
                obj.sensor_placement(:,i) = [x; y; theta];
            end                        
            obj.calibrated = true;
        end
        
        function p = ob_vector (p1, p2)
            p = (p2 - p1)/norm(p2-p1);
        end
    end
    
end
