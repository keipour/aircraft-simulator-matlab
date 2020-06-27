function omega_dot = GetAngularAcceleration(obj, moment)
    omega_dot = obj.I_inv * (moment - cross(obj.State.Omega, obj.I * obj.State.Omega));
end
