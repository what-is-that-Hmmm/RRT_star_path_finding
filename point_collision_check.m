function feasible = point_collision_check(point,map)
    feasible=true;
    %   check if it is a point in the map
    if ~(point(1)>=1 && point(1)<=size(map,2) && ... % x in map
         point(2)>=1 && point(2)<=size(map,1)) % y in map
        feasible=false;
    end
    
    if feasible
        if (map(floor(point(2)),floor(point(1)))~=255 || ...
            map(ceil(point(2)),ceil(point(1)))~=255)
            feasible=false;
        end
    end
end

