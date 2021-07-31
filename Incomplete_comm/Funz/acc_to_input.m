function [Thrust,Pitch,Roll] = acc_to_input(acceleration,yaw,m)

    Gravity = 9.81;
        
    Pitch = atan2((-acceleration(1)*cos(yaw)-acceleration(2)*sin(yaw)), (acceleration(3) + Gravity));
    Roll = atan2(((-acceleration(1)*sin(yaw)+acceleration(2)*cos(yaw))*cos(Pitch)), (acceleration(3)+Gravity));
    
    Thrust = m*(acceleration(3)+Gravity)/(cos(Pitch)*cos(Roll));

end