function obj = UpdateState(obj, RotorSpeedsSquared, dt)
    % Calculate the total force and moment
    obj.State.Force = multirotor.GetGravityForce(obj) + ...
        multirotor.GetThrustForce(obj, RotorSpeedsSquared);
    obj.State.Moment = multirotor.GetGravityMoment(obj) + ...
        multirotor.GetThrustMoment(obj, RotorSpeedsSquared) + ...
        multirotor.GetReactionMoment(obj, RotorSpeedsSquared);

    % Calculate the equations of motion
    p_dotdot = multirotor.GetLinearAcceleration(obj, obj.State.Force);
    omega_dot = multirotor.GetAngularAcceleration(obj, obj.State.Moment);
    phi_dot = multirotor.GetEulerDerivative(obj);

    % Update the rest of the state
    obj.State.Position = obj.State.Position + 0.5 * obj.State.Acceleration * dt * dt + ...
        obj.State.Velocity * dt;
    obj.State.Velocity = obj.State.Velocity + obj.State.Acceleration * dt;
    obj.State.Acceleration = p_dotdot;

    obj.State.RPY = wrapTo180(obj.State.RPY + obj.State.EulerDerivative * dt);
    obj.State.Omega = obj.State.Omega + obj.State.AngularAcceleration * dt;
    obj.State.EulerDerivative = phi_dot;
    obj.State.AngularAcceleration = omega_dot;
end
