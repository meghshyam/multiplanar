function [angle] = findAngle(plane_normal_1, plane_normal_2)

    plane_normal_1
    plane_normal_2
	dotProduct = dot(plane_normal_1, plane_normal_2);
	mag1 =  sqrt( plane_normal_1(1)*plane_normal_1(1) + ...
                    plane_normal_1(2)*plane_normal_1(2) + ...
                    plane_normal_1(3)*plane_normal_1(3) );
	mag2 =  sqrt( plane_normal_2(1)*plane_normal_2(1) + ...
                    plane_normal_2(2)*plane_normal_2(2) + ...
                    plane_normal_2(3)*plane_normal_2(3) );
	angle = acos(dotProduct/(mag1*mag2));
	crossProduct = cross(plane_normal_1, plane_normal_2);
	ref = [0,0,1];
	sign = dot(ref, crossProduct);
	if sign < 0
        angle = -angle;
    end
    angle = (angle*180) / 3.14159;
end