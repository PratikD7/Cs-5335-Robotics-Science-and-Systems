function pt_position = checkLineIntersection(pt1, pt2, pt3)
    if pt1(1) == pt2(1)             
        if pt3(1) == pt1(1)
            pt_position = 2;           
        else
            if pt3(1) > pt1(1)
                pt_position = 1;       
            else
                pt_position = 0;       
            end
        end
    else                       
        NVer = (pt1(2) - pt2(2))/(pt1(1) - pt2(1))*pt3(1) + (pt1(1)*pt2(2) - pt1(2)*pt2(1))/(pt1(1) - pt2(1));
        if pt3(2) == NVer
            pt_position = 2;           
        else
            if pt3(2) > NVer
                pt_position = 1;       
            else
                pt_position = 0;       
            end
        end
    end
end