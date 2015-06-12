%%

% First we must define some parameters.

% Maximum speed for an F1 car, assuming very long straight.
v_max = 100;
% Mass of an F1 car + driver + half a tank of gasoline.
m = 750;
% Maximum power for an F1 engine is approximately 550 kW (~ 740 hp).
P_max = 550e3;
% Peak braking de-acceleration for an F1 car is approximately 5 G. One can
% estimate that 0.5 G is due to air resistance, and 4.5 G due to the
% breaks. The tyre friction coefficient for a racing tyre is approximately
% 1.2, so, peak effective mass is 4.5/1.2 times the mass. Downforce is this
% minus 1. Maybe 80% of the mass can be translated to rear axis during
% acceleration.
f_grip = @(v) 9.8 * 1.5 * m * (1 + ((5-.5)/1.5 - 1) * v/v_max);
f_grip_acceleration = @(v) .8 * f_grip(v);
% Air resistance at maximum speed equals the peak power, and is
% proportional to the square of speed.
f_air = @(v) (v / v_max).^2 * (P_max/v_max);
% The acceleration of the F1 car is:
a_braking = @(v) -(v >= 0) .* (f_grip(v) + f_air(v)) / m;
a_acceleration = @(v, throttle) ...
    (min(f_grip_acceleration(v), throttle .* P_max ./ v) - f_air(v)) / m;
% The fuel flow rate is:
fuel_flow_rate = @(v, throttle) ...
    min(f_grip(v), throttle .* P_max ./ v) .* v / P_max * (100 / 3600);

% We are being lazy, and not bothering to interpolate the results. Let us
% have a millisecond resolution without interpolation. This is
% computationally more expensive, but definitely not computationally
% expensive on modern hardware.
options = odeset('MaxStep', .001);

%%

% Feasibility testing for the parameters.
v = linspace(0,v_max);

figure(1); clf;
plot(3.6 * v, 1e-3 * f_grip_acceleration(v), ...
    3.6 * v, 1e-3 * f_grip(v), ...
    3.6 * v, 1e-3 * (P_max ./ v), ...
    3.6 * v, 1e-3 * f_air(v));
xlim([0 360]);
ylim([0 50]);
xlabel('velocity (kph)');
ylabel('force (kN)');
legend({'traction limit (acceleration)', 'traction limit (braking)', ...
    'power limit', 'air resistance'});

figure(2); clf;
plot(3.6 * v, (1/9.8) * a_acceleration(v, 1), ...
    3.6 * v, (1/9.8) * a_braking(v));
xlim([0 360]);
ylim([-6 4]);
xlabel('velocity (kph)');
ylabel('peak acceleration (g)');
legend({'accelerating', 'braking'});

%%
% A 1000-m straight, starting from 60 km/h ending to 80 km/h. How fast we
% are?

% Where is our braking point? And, how does it depend on our velocity.
solution_braking = ode45(@(t,v) [v(2); -a_braking(v(2))], [0 3], [0 80/3.6], options);
braking_distance = @(v) interp1(solution_braking.y(2,:), solution_braking.y(1,:), v);
braking_time = @(v) interp1(solution_braking.y(2,:), solution_braking.x, v);
must_brake = @(v,s) (1000 - s) < braking_distance(v);

% One can also compute the 0-to-100-kph times. But, this is left as an
% exercise to the reader.

%%

% First, with full throttle.

solution = ode45(@(t,v) [v(2);a_acceleration(v(2),1)], [0 20], [0 60/3.6], options);
ind = find(must_brake(solution.y(2,:),solution.y(1,:)), 1);
t_b1 = solution.x(ind);
s_b1 = solution.y(1,ind);
v_b1 = solution.y(2,ind);
jnd = find(solution_braking.x >= braking_time(v_b1), 1);

t_t1 = t_b1 + braking_time(v_b1);
m_c1 = trapz(solution.x(:,1:ind), fuel_flow_rate(solution.y(2,1:ind),1));
fprintf('It takes %.3f s, with top speed of %.0f km/h (fuel usage %.0f g).\n', t_t1, 3.6 * v_b1, 1000 * m_c1);

figure(3); clf; hold('on');
plot([solution.x(1:ind) solution.x(ind) + solution_braking.x(jnd) - solution_braking.x(jnd:-1:1)], [solution.y(1,1:ind) solution.y(1,ind) + solution_braking.y(1,jnd) - solution_braking.y(1,jnd:-1:1)]);
plot(t_b1 + [0 braking_time(v_b1)], s_b1 + [0 braking_distance(v_b1)],'+');
figure(4); clf; hold('on');
plot([solution.x(1:ind) solution.x(ind) + solution_braking.x(jnd) - solution_braking.x(jnd:-1:1)], 3.6 * [solution.y(2,1:ind) solution_braking.y(2,jnd:-1:1)]);
plot(t_b1 + [0 braking_time(v_b1)], 3.6 * [v_b1 solution_braking.y(2,1)],'+');

% Then lifting one second earlier.

solution = ode45(@(t,v) [v(2);a_acceleration(v(2),t <= t_b1 - 1)], [0 20], [0 60/3.6], options);
ind_lift = find(solution.x >= t_b1 - 1);
ind = find(must_brake(solution.y(2,:),solution.y(1,:)),1);
t_b2 = solution.x(ind);
s_b2 = solution.y(1,ind);
v_b2 = solution.y(2,ind);
jnd = find(solution_braking.x >= braking_time(v_b2), 1);

t_t2 = t_b2 + braking_time(v_b2);
m_c2 = trapz(solution.x(:,1:ind_lift), fuel_flow_rate(solution.y(2,1:ind_lift),1));
fprintf('Lifting before previous braking point, it takes %.3f s more, with top speed of %.0f km/h (fuel saved %.0f g).\n', t_t2 - t_t1, 3.6 * max(solution.y(2,:)), 1000 * (m_c1 - m_c2));

figure(3);
plot([solution.x(1:ind) solution.x(ind) + solution_braking.x(jnd) - solution_braking.x(jnd:-1:1)], [solution.y(1,1:ind) solution.y(1,ind) + solution_braking.y(1,jnd) - solution_braking.y(1,jnd:-1:1)]);
plot([t_b1 - 1 t_b2 t_b2 + braking_time(v_b2)], [interp1(solution.x,solution.y(1,:), t_b1 - 1) s_b2 s_b2 + braking_distance(v_b2)],'+');
figure(4);
plot([solution.x(1:ind) solution.x(ind) + solution_braking.x(jnd) - solution_braking.x(jnd:-1:1)], 3.6 * [solution.y(2,1:ind) solution_braking.y(2,jnd:-1:1)]);
plot([t_b1 - 1 t_b2 t_b2 + braking_time(v_b2)], 3.6 * [interp1(solution.x,solution.y(2,:), t_b1 - 1) v_b2 solution_braking.y(2,1)],'+');

% Then cutting engine power.

throttle = .9845;
solution = ode45(@(t,v) [v(2);a_acceleration(v(2),throttle)], [0 20], [0 60/3.6], options);
ind = find(must_brake(solution.y(2,:),solution.y(1,:)),1);
t_b3 = solution.x(ind);
s_b3 = solution.y(1,ind);
v_b3 = solution.y(2,ind);
jnd = find(solution_braking.x >= braking_time(v_b3), 1);

t_t3 = t_b3 + braking_time(v_b3);
m_c3 = trapz(solution.x(:,1:ind), fuel_flow_rate(solution.y(2,1:ind),throttle));
fprintf('Cutting power, it takes %.3f s more, with top speed of %.0f km/h (fuel saved %.1f g).\n', t_t3 - t_t1, 3.6 * v_b3, 1000 * (m_c1 - m_c3));

figure(3);
plot([solution.x(1:ind) solution.x(ind) + solution_braking.x(jnd) - solution_braking.x(jnd:-1:1)], [solution.y(1,1:ind) solution.y(1,ind) + solution_braking.y(1,jnd) - solution_braking.y(1,jnd:-1:1)]);
plot(t_b3 + [0 braking_time(v_b3)], s_b3 + [0 braking_distance(v_b3)],'+');
xlabel('time (s)');
ylabel('position (m)');
ylim([0 1001]);
figure(4);
plot([solution.x(1:ind) solution.x(ind) + solution_braking.x(jnd) - solution_braking.x(jnd:-1:1)], 3.6 * [solution.y(2,1:ind) solution_braking.y(2,jnd:-1:1)]);
plot(t_b3 + [0 braking_time(v_b3)], 3.6 * [v_b3 solution_braking.y(2,1)],'+');
xlabel('time (s)');
ylabel('velocity (kph)');
ylim([0 350]);
