function [ d1, d2, c1_2, c2_1 ] = ray_interact( r1, r2, t1, t2 )
%find the interaction of two line 
%   input: vetex and ray.
%   output: depth and closet points on both rays

        n1_2 = cross(r1, cross(r1,r2));
        n2_1 = cross(r2, cross(r2,r1));
        
        c1_2 = t1 + (((t2 - t1)'*n2_1)/(r1'*n2_1))*r1;
        c2_1 = t2 + (((t1 - t2)'*n1_2)/(r2'*n1_2))*r2;
        
%         d1 = norm(c1_2 - t1);
%         d2 = norm(c2_1 - t2);
        d1 = (c1_2-t1)'*r1;
        d2 = (c2_1 - t2)'*r2;
end