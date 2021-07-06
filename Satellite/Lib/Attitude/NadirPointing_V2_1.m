function eul_des = NadirPointing_V2_1(axis)


switch axis
    
    case 'X'
        
        eul_des = [0; 0; 0]; 
        
    case 'Y'
        
        eul_des = [pi/2; 0; 0];
        
    case 'Z'
        
        eul_des = [0; pi/2; 0];
        
end


end