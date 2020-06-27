function M = GetGravityMoment(obj)
    M = zeros(3, 1);
    for i = 1 : obj.NumOfRotors
        r = obj.Rotors{i}.Position;
        G_motor = obj.Rotors{i}.MotorMass * obj.Gravity;
        G_motorB = obj.Rotors{i}.R * G_motor;
        G_arm = obj.Rotors{i}.ArmMass * obj.Gravity;
        G_armB = obj.Rotors{i}.R * G_arm;
        M = M + cross(r, G_motorB) + cross(r/2, G_armB);
    end
end
