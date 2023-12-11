classdef Vehicle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        dimension = [2 3]; %[2.09 2.87]; % [width, length]
        
        wheelDist = 2;
        steeringRatio = 5; % k <-- w = v * sin(theta / k) * l
        
        name = '';
        
        status; % [X, Y, Z, v, a, h, w]
        % x, y, z: in meters
        % v: m/s
        % a: m/s2
        % h: rad north as 0 clock-wise
        % w: rad/s
        
        accMax = 0.5 * 9.8;
        %accMin = -0.5 * 9.8;
        accMin = -1;
        
    end
    
    properties (Constant)
        STATUS_LOC_X = 1;
        STATUS_LOC_Y = 2;
        STATUS_LOC_Z = 3;
        STATUS_V     = 4;
        STATUS_A     = 5;
        STATUS_H     = 6;
        STATUS_W     = 7;
        STATUS_TIME  = 8;
        STATUS_TOTAL = 8;
        
        DEBUG_PLOT = 0;
    end
    
    methods
        function obj = Vehicle(status)
        % Constructor
        
            obj.status = status;
        
        end
        
        
    end
    
    methods (Static)
        
        %%%% Main functions
        
        function [t, distMin, distMax, dV, dA, vertCoor] = computeTimeToCollision(ve, v1)
            t = nan;
            
            % Change to ego vehicle's coordinate system
            h1 = wrapToPi(v1.status(1,Vehicle.STATUS_H) - ve.status(1,Vehicle.STATUS_H));            
            
            % Compute the vertices of vehicle 1
            relLoc = v1.status(1,Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y) - ve.status(1,Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y);
            offset = Vehicle.rotateCoor(relLoc, -ve.status(1,Vehicle.STATUS_H));
            vertCoor = Vehicle.computeVehicleVertices(h1, v1.dimension(1), v1.dimension(2), offset);
            
            minCoor = min(vertCoor);
            maxCoor = max(vertCoor);
            
            % Compute min dist
            hWidth = ve.dimension(1) / 2;
            hLength = ve.dimension(2) / 2;
            
            if minCoor(2) > hLength
                distYMin = minCoor(2) - hLength;
            elseif maxCoor(2) < -hLength
                distYMin = maxCoor(2) + hLength;
            else
                distYMin = 0;
            end
            
            if minCoor(1) > hWidth
                distXMin = minCoor(1) - hWidth;
            elseif maxCoor(1) < -hWidth
                distXMin = maxCoor(1) + hWidth;
            else
                distXMin = 0;
            end
            
            distXMax = max([abs(maxCoor(1) + hWidth) abs(minCoor(1) - hWidth)]) * sign(distXMin);
            distYMax = max([abs(maxCoor(2) + hLength) abs(minCoor(2) - hLength)]) * sign(distYMin);
            
            % Plot Vehicles
            if Vehicle.DEBUG_PLOT == 1
                figure()                
                vertCoorEgo = Vehicle.computeVehicleVertices(0, ve.dimension(1), ve.dimension(2), [0 0]);
                
                Vehicle.plotRect(vertCoor, 'v1');
                Vehicle.plotRect(vertCoorEgo, 've');
                grid on
                axis equal
            end
            
            % Compute expected overlapped interval for X and Y direction
            dV = ve.status(Vehicle.STATUS_V) * [0 1] - v1.status(Vehicle.STATUS_V) * [sin(h1) cos(h1)];
            dA = ve.status(Vehicle.STATUS_A) * [0 1] -v1.status(Vehicle.STATUS_A) * [sin(h1) cos(h1)];
            
            if (sign(distXMin) * sign(dV(1)) < 0) && (sign(dV(1)) == sign(dA(1)) || sign(dA(1)) == 0)
                return;
            elseif (sign(distYMin) * sign(dV(2)) < 0) && (sign(dV(2)) == sign(dA(2)) || sign(dA(2)) == 0)
                return;
            end
            
            distMin = [distXMin distYMin];
            distMax = [distXMax distYMax];
            DMin = dV .^ 2 + 2 * dA .* distMin;
            DMax = dV .^ 2 + 2 * dA .* distMax;
            
            if (distXMin ~= 0) && (distYMin ~= 0)
                % The regular case                
                
                if sum(DMin < 0) > 0
                    return;
                end                                
                
                tCX = [Vehicle.computeTimeToCollisionCloseForm(DMin(1), dV(1), dA(1), distMin(1)), Vehicle.computeTimeToCollisionCloseForm(DMax(1), dV(1), dA(1), distMax(1))];
                tCY = [Vehicle.computeTimeToCollisionCloseForm(DMin(2), dV(2), dA(2), distMin(2)), Vehicle.computeTimeToCollisionCloseForm(DMax(2), dV(2), dA(2), distMax(2))];
                
                t = Vehicle.findTimeOverlap(tCX, tCY);
                
            elseif distXMin == 0
                
                if DMin(2) < 0
                    return;
                end 
                
                tCY = [Vehicle.computeTimeToCollisionCloseForm(DMin(2), dV(2), dA(2), distMin(2)), Vehicle.computeTimeToCollisionCloseForm(DMax(2), dV(2), dA(2), distMax(2))];
                t = tCY;
                
            elseif distYMin == 0
                
                if DMin(1) < 0
                    return;
                end
                
                tCX = [Vehicle.computeTimeToCollisionCloseForm(DMin(1), dV(1), dA(1), distMin(1)), Vehicle.computeTimeToCollisionCloseForm(DMax(1), dV(1), dA(1), distMax(1))];
                t = tCX;
            end
            
        end
        
        function t = computeTimeToCollisionCloseForm(D, v, a, dist)
            
            if D < 0
                t = nan;
                return;
            end
            
            if a ~= 0
                temp = (-v + [-sqrt(D) sqrt(D)]) / a;
            else
                temp = dist / v;
                if temp > 0
                    t = temp;
                else
                    t = nan;
                end
                return;
            end
            
            if (temp(1) >= 0) && (temp(2) >= 0)
                t = temp(1);
            elseif (temp(1) >= 0)
                t =temp(1);
            elseif (temp(2) >= 0)
                t =temp(2);
            else
                t = nan;
            end
            
        end
        
        function [t, willCollide] = computeTimeToAvoidCollision(ve, v1, distMin, distMax, dV, dA, vertCoor)
            t = nan;
            willCollide = false;
            
            h1 = wrapToPi(v1.status(1,Vehicle.STATUS_H) - ve.status(1,Vehicle.STATUS_H));
            a1 = v1.status(Vehicle.STATUS_A) * Vehicle.hToVec(h1);
            v1Y = v1.status(Vehicle.STATUS_V) * cos(h1);
            a1Y = a1(2);
            
            veY = ve.status(Vehicle.STATUS_V);
            ae = ve.status(Vehicle.STATUS_A);
            am = ve.accMin;                        
            
            % Acceleration and deceleration only
            
            % First check if there exist a feasible solution
            tAll = (veY - v1Y) / (-am);
            de = veY * tAll + 0.5 * am * tAll ^ 2;
            d1 = v1Y * tAll + 0.5 * a1Y * tAll ^ 2;
            if de - d1 > distMin(2)
                t = nan;
                willCollide = true;
                return;
            end
            
            if ve.status(Vehicle.STATUS_W) == 0
                
                p = -dV(2) / (am - a1Y);
                q = -dA(2) / (am - a1Y);
                
                A = 0.5 * ae + q * ae + 0.5 * am * q ^ 2 - 0.5 * a1Y * (q + 1) ^ 2;
                B = veY + q * veY + p * ae + p * q * am - (v1Y + a1Y * p) * (q + 1);
                C = veY * p + 0.5 * am * p ^ 2 - distMin(2) - v1Y * p - 0.5 * a1Y * p ^2;
                
                if A ~= 0
                    D = B ^ 2 - 4 * A * C;
                    if D < 0
                        t = nan;
                    else
                        temp = (- B + [-sqrt(D) sqrt(D)]) / (2 * A);

                        if (temp(1) >= 0) && (temp(2) >= 0)
                            t = temp(1);
                        elseif (temp(1) >= 0)
                            t =temp(1);
                        elseif (temp(2) >= 0)
                            t =temp(2);
                        else
                            t = nan;
                        end
                    end
                else                    
                    temp = - C / B;
                    if temp < 0
                        t = nan;
                    else
                        t = temp;
                    end                    
                end
                
            end
            
            % 
            
        end                                
        
        function [vehicleNew] = performPredictionNoSteering(vehicle, t, varargin) 
            
            hVec = Vehicle.hToVec(vehicle.status(Vehicle.STATUS_H));
            
            v = vehicle.status(Vehicle.STATUS_V) * hVec;
            a = vehicle.status(Vehicle.STATUS_A) * hVec;
            
            loc = vehicle.status(Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y) + v * t + 0.5 * a * t ^ 2;
            
            vehicleNew = copy(vehicle);
            vehicleNew.status(Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y) = loc;
            
            if ~isempty(varargin)
                
                figure
                hold on
                                
                vehicleNew.name = [vehicle.name ' New'];
                vehicle.name = [vehicle.name ' Old'];
                Vehicle.plotVehicle(vehicle);
                Vehicle.plotVehicle(vehicleNew);
                
                hold off
                axis equal
                grid on
            end
            
        end
        
        function yawArr = computeYawRateFromSteering(vehicle, speedArr, steeringArr)
                        
            yawArr = speedArr .* sin(steeringArr / vehicle.steeringRatio) * vehicle.wheelDist;
            
        end
        
    end
    
    methods (Static)
        
        %%%% Helper functions
        
        function [dist, dir] = getDistDir(ve, v1)
            
            coorDiff = v1.status(1,Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y) - ve.status(1,Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y);
            dist = norm(coorDiff);
            dir = atan2(coorDiff(2), coorDiff(1)) - ve.status(1,Vehicle.STATUS_H);
            dir = wrapToPi(dir);
        end
        
        function relLosV = getRelativeLosSpeed(ve, v1)
            
            relV = v1.status(Vehicle.STATUS_V) * Vehicle.hToVec(v1.status(Vehicle.STATUS_H)) - ve.status(Vehicle.STATUS_V) * Vehicle.hToVec(ve.status(Vehicle.STATUS_H));
            
            coorDiff = v1.status(1,Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y) - ve.status(1,Vehicle.STATUS_LOC_X:Vehicle.STATUS_LOC_Y);
            relDirCoor = coorDiff / norm(coorDiff);
            
            relLosV = dot(relV, relDirCoor);
            
        end
        
        function vec = hToVec(h)
            
            vec = [sin(h), cos(h)];
        end
        
        function vertArr = computeVehicleVertices(headingDiff, width, length, offset)
            
            vertArr = [-width/2 length/2; width/2 length/2; -width/2 -length/2; width/2 -length/2];
            %rotMat = [cos(heading) -sin(heading); sin(heading) cos(heading)];            
            vertArr = Vehicle.rotateCoor(vertArr, headingDiff) + [offset; offset; offset; offset];
            
        end
        
        function arr = rotateCoor(oriArr, theta)
            
            rotMat = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            arr = oriArr * rotMat;
            
        end
        
        function plotRect(vertArr, varargin)
            
            if ~isempty(varargin)
                txt = varargin{1};
            else
                txt = '';
            end
            
            hold on
            plot(vertArr([1 2], 1), vertArr([1 2], 2), 'r')
            plot(vertArr([1 3], 1), vertArr([1 3], 2), 'k')
            plot(vertArr([1 4], 1), vertArr([1 4], 2), 'k')
            plot(vertArr([2 3], 1), vertArr([2 3], 2), 'k')
            plot(vertArr([2 4], 1), vertArr([2 4], 2), 'k')
            plot(vertArr([3 4], 1), vertArr([3 4], 2), 'k')
            
            if ~isempty(txt)
                text(mean(vertArr([1 4],1)), mean(vertArr([1 4],2)), ['\leftarrow' txt]);
            end
            
            hold off
        end        
        
        function tArr = findTimeOverlap(arr1, arr2)
            
            tStart = max([arr1(1) arr2(1)]);
            tEnd = min([arr1(2) arr2(2)]);
            
            if tEnd > tStart
                tArr = [tStart tEnd];
            else
                tArr = nan;
            end
            
        end
        
        function plotVehicle(vehicle)
            vertCoor = Vehicle.computeVehicleVertices(vehicle.status(Vehicle.STATUS_H), vehicle.dimension(1), vehicle.dimension(2), [vehicle.status(Vehicle.STATUS_LOC_X) vehicle.status(Vehicle.STATUS_LOC_Y)]);                                
            Vehicle.plotRect(vertCoor, vehicle.name);
        end
        
    end
    
end

