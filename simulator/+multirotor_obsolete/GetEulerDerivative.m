function phi_dot = GetEulerDerivative(obj)
    sphi = sind(obj.State.RPY(1));
    cphi = cosd(obj.State.RPY(1));
    ttheta = tand(obj.State.RPY(2));
    ctheta = cosd(obj.State.RPY(2));
    eta = [1,   sphi*ttheta, cphi*ttheta;
           0, cphi, -sphi;
           0, sphi / ctheta, cphi / ctheta];
    phi_dot = rad2deg(eta * obj.State.Omega);
end
