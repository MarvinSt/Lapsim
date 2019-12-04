

function [solution,times,solver] = lapsim


% Specify  curvature
dist = [ 0  200 400  600   800  1000 ] / 4;
curv = [ 0  0.03 0	-0.06    0 0 ];

MAX_DIST = max(dist);
NUM_ITT = 100;
VEC_DIST = linspace(0, MAX_DIST, NUM_ITT);

Cs = interp1(dist, curv, VEC_DIST);


% Reconstruct the track
phi     = cumsum(Cs .* gradient(VEC_DIST));
xp      = cumsum(cos(phi) .* gradient(VEC_DIST));
yp      = cumsum(sin(phi) .* gradient(VEC_DIST));
xu      = xp + sin(phi) * 2;
yu      = yp - cos(phi) * 2;
xb      = xp - sin(phi) * 2;
yb      = yp + cos(phi) * 2;

figure(1)
figure
plot(xp, yp, '--k')
hold on
plot(xu, yu, 'k')
plot(xb, yb, 'k')
hold off
grid on
drawnow



solver = ocl.Problem(NUM_ITT, @varsfun, @daefun, @pathcosts, ...
    'gridconstraints', @gridconstraints, ...
    'N', NUM_ITT);





solver.setParameter('ds', MAX_DIST / NUM_ITT);

solver.setParameter('m', 900);
solver.setParameter('kt_drive', 0.45);
solver.setParameter('kt_brake', 0.6);
solver.setParameter('kd', 10);
solver.setParameter('cx', 0.8);
solver.setParameter('cz', 1.0);
solver.setParameter('rho', 1.22);
solver.setParameter('sf', 1.6);
solver.setParameter('sr', 1.6);
solver.setParameter('wbal', 0.5);
solver.setParameter('abal', 0.45);
solver.setParameter('rball', 0.5);
solver.setParameter('lf', 1.5);
solver.setParameter('lr', 1.5);
solver.setParameter('hcg', 0.35);
solver.setParameter('re', 0.33);
solver.setParameter('g', 9.81);
solver.setParameter('Izz', 1400);
solver.setParameter('Iw', 10);
solver.setParameter('pcx1', 1.6);
solver.setParameter('pcy1', 1.6);


solver.setParameter('tq_gain', 1000);

% solver.setBounds('ey', -3, 3);
% solver.setBounds('vx', 5, 150);
% solver.setBounds('vy', -15, 15);

solver.setBounds('Cs', Cs);
% solver.setBounds('time', 0, 20);

% solver.setInitialBounds('vx',   5.0);
% solver.setInitialBounds('vy',   0.0);
% solver.setEndBounds('vx',  5.0 );
% solver.setEndBounds('vy',  0.0 );

initialGuess    = solver.getInitialGuess();

% Initialize the middle lane
% N        = length(initialGuess.states.x.value);
% x_road   = linspace(0,2*pi,N);
% y_center = sin(x_road);


initialGuess.states.vx.set(10 * ones(size(initialGuess.states.vx.value)));

% initialGuess.states.time.set(cumsum(1 ./ initialGuess.states.vx.value));

% initialGuess.states.y.set(y_center);

% Solve OCP
[solution,times] = solver.solve(initialGuess);

% Reconstruct the track
phi     = cumsum(Cs .* gradient(VEC_DIST));
xp      = cumsum(cos(phi) .* gradient(VEC_DIST));
yp      = cumsum(sin(phi) .* gradient(VEC_DIST));
xu      = xp + sin(phi) * 2;
yu      = yp - cos(phi) * 2;
xb      = xp - sin(phi) * 2;
yb      = yp + cos(phi) * 2;

vx      = solution.states.vx.value;
ey      = solution.states.ey.value;
vx      = vx(1:end-1);
ey      = ey(1:end-1);

xcg     = xp - sin(phi) .* ey;
ycg     = yp + cos(phi) .* ey;


% Plot solution
figure(1)
figure
plot(xp, yp, '--k')
hold on
plot(xu, yu, 'k')
plot(xb, yb, 'k')
plot3(xcg,ycg,vx)
hold off
grid on




% Plot solution
% figure('units','normalized')
% subplot(3,2,1);hold on;grid on;
% plot(times.states.value,solution.states.vx.value,'Color','b','LineWidth',1.5);
% plot(times.states.value,solution.states.ey.value,'Color','r','LineWidth',1.5);
% ylabel('[m]');legend({'x','y'});
% 
% subplot(3,2,3);hold on;grid on;
% vx = solution.states.vx.value;
% vy = solution.states.vy.value;
% V  = sqrt(vx.^2+vy.^2);
% 
% plot(times.states.value,vx,'Color','b','LineWidth',1.5);
% plot(times.states.value,vy,'Color','r','LineWidth',1.5);
% plot(times.states.value,V,'Color','g','LineWidth',1.5);
% legend({'vx','vy','V'});
% % plot(times.states.value,Vmax.*ones(1,length(times)),'Color','k','LineWidth',1.5,'LineStyle','-.')
% ylabel('[m/s]');

% subplot(3,2,5);hold on;grid on;
% plot(times.states.value,solution.states.Fx.value,'Color','b','LineWidth',1.5)
% plot(times.states.value,solution.states.Fy.value,'Color','r','LineWidth',1.5)
% legend({'Fx','Fy'});
% plot(times.states.value,-Fmax.*ones(1,length(times.states.value)),'Color','k','LineWidth',1.5,'LineStyle','-.')
% plot(times.states.value, Fmax.*ones(1,length(times.states.value)),'Color','k','LineWidth',1.5,'LineStyle','-.')
% ylabel('[N]');xlabel('time');
% 
% % build street
% subplot(3,2,[2,4,6]);hold on;grid on;
% x_road   = linspace(0,2*pi,1000);
% y_center = sin(x_road);
% 
% y_max = y_center + road_bound;
% y_min = y_center - road_bound;
% 
% plot(x_road,y_center,'Color','k','LineWidth',0.5,'LineStyle','--');
% plot(x_road,y_min   ,'Color','k','LineWidth',0.5,'LineStyle','-');
% plot(x_road,y_max   ,'Color','k','LineWidth',0.5,'LineStyle','-');
% plot(solution.states.x.value,...
%     solution.states.y.value,'Color','b','LineWidth',1.5);
% axis equal;xlabel('x[m]');ylabel('y[m]');
% 
% % Show Animation
% animate(times.states.value,solution,x_road,y_center,y_min,y_max)

end

function varsfun(sh)
sh.addState('epsi', 'lb', -deg2rad(30), 'ub', deg2rad(30));
sh.addState('ey', 'lb', -2, 'ub', 2);
sh.addState('vx', 'lb', 5, 'ub', 150);
sh.addState('vy', 'lb', -10, 'ub', 10);
sh.addState('dpsi', 'lb', -1, 'ub', 1);

sh.addState('wfl', 'lb', 1);
sh.addState('wfr', 'lb', 1);
sh.addState('wrl', 'lb', 1);
sh.addState('wrr', 'lb', 1);

sh.addState('del_f', 'lb', -deg2rad(30), 'ub', deg2rad(30));
sh.addState('tq_tot', 'lb', -100, 'ub', 50);

% sh.addState('time');
% sh.addState('dist');
  


sh.addControl('ddel_f');
sh.addControl('dtq_tot');

sh.addControl('Cs');



sh.addParameter('ds');


sh.addParameter('m');
sh.addParameter('kt_drive');
sh.addParameter('kt_brake');
sh.addParameter('kd');
sh.addParameter('cx');
sh.addParameter('cz');
sh.addParameter('rho');
sh.addParameter('sf');
sh.addParameter('sr');
sh.addParameter('wbal');
sh.addParameter('abal');
sh.addParameter('rball');
sh.addParameter('lf');
sh.addParameter('lr');
sh.addParameter('hcg');
sh.addParameter('re');
sh.addParameter('g');
sh.addParameter('Izz');
sh.addParameter('Iw');

sh.addParameter('pcx1');
sh.addParameter('pcy1');

sh.addParameter('tq_gain');
end


% -------------------------------------------------------------------------
% ### TIRE MODEL
% -------------------------------------------------------------------------
function [fx, fy] = TireModel(sx, sy, fx_max, fy_max, sx_scl, sy_scl, pcx1, pcy1)
% Normalize slip
sxn     = sx .* sx_scl;
syn     = sy .* sy_scl;
scn     = sqrt(sxn .* sxn + syn .* syn);

% Calculate force
fx      = sxn ./ (scn + 1e-5) .* fx_max .* sin(pcx1 .* atan(scn ./ pcx1));
fy      = syn ./ (scn + 1e-5) .* fy_max .* sin(pcy1 .* atan(scn ./ pcy1));
end


function y = softplus(x)
y = log(1 + exp(x));
end

function y = softminus(x)
y = -log(1 + exp(-x));
end

function daefun(sh,x,~,u,p)
% -------------------------------------------------------------------------
% ### TIME TO DISTANCE
% -------------------------------------------------------------------------
% Compute time to distance variable
dsdt  	= (x.vx * cos(x.epsi) - x.vy * sin(x.epsi)) / (1 - x.ey .* u.Cs);
% ds = 1;

% -------------------------------------------------------------------------
% ### STEERING ANGLES
% -------------------------------------------------------------------------
% Get steering angles
delta   = [ x.del_f
    x.del_f
    0*x.del_f
    0*x.del_f
    ];
cos_del = cos(delta);
sin_del = sin(delta);

% -------------------------------------------------------------------------
% ### TIRE VELOCITY & SLIP CALCULATION
% -------------------------------------------------------------------------
% Calculate body frame velocities
vx_b    = repmat(x.vx, 4, 1) + [ p.sf	-p.sf	p.sr	-p.sr	].' * x.dpsi;
vy_b    = repmat(x.vy, 4, 1) + [ p.lf    p.lf    -p.lr 	-p.lr	].' * x.dpsi;

% Eexact cos & sin angles
vx_w    = cos_del .* vx_b + sin_del .* vy_b;
vy_w    = cos_del .* vy_b - sin_del .* vx_b;

% Get wheels
vwhl    = [ x.wfl x.wfr x.wrl x.wrr ].';

% Slip calculation
sx      = (vwhl - vx_w) ./ vwhl;
sy      = -vy_w ./ vwhl;

% -------------------------------------------------------------------------
% ### VERTICAL LOAD
% -------------------------------------------------------------------------
% Aerodynamic forces
fl      = 0.5 * p.rho * p.cz * (x.vx .* x.vx);
fd      = 0.5 * p.rho * p.cx * x.vx .* abs(x.vx);

fz      = [ 1 1 1 1 ].' * p.m * p.g / 4;

% -------------------------------------------------------------------------
% ### TIRE FORCES
% -------------------------------------------------------------------------
% Compute maximum force and slip stiffness
kx_nom  = 30.0 * fz;
ky_nom  = 35.0 * fz;
fx_max  = 1.60 * fz;
fy_max  = 1.55 * fz;

% Compute slip scaling factors
sx_scl  = kx_nom ./ fx_max;
sy_scl  = ky_nom ./ fy_max;

% Compute tire forces
[fxt, fyt] = TireModel(sx, sy, fx_max, fy_max, sx_scl, sy_scl, p.pcx1, p.pcy1);

% Transform tire to body forces
fxb     = cos_del .* fxt - sin_del .* fyt;
fyb     = cos_del .* fyt + sin_del .* fxt;

% Calculate sum of forces
fx      = sum(fxb);
fy      = sum(fyb);
mz      = p.lf * (fyb(1, :) + fyb(2, :)) - p.lr * (fyb(3, :) + fyb(4, :)) ...
    + p.sf * (fxb(1, :) - fxb(2, :)) + p.sr * (fxb(3, :) - fxb(4, :));

% -------------------------------------------------------------------------
% ### TORQUE DISTRIBUTION
% -------------------------------------------------------------------------
% Compute drive and brake torque
% tq_front    = max(x.tq_tot, 0) * kt_drive + min(x.tq_tot, 0) * kt_brake;
% tq_rear     = max(x.tq_tot, 0) * (1 - kt_drive) + min(x.tq_tot, 0) * (1 - kt_brake);
% tq_front    = ((x.tq_tot >= 0) * p.kt_drive + (x.tq_tot < 0) * p.kt_brake) * x.tq_tot;
% tq_rear     = ((x.tq_tot >= 0) *(1 - p.kt_drive) + (x.tq_tot < 0) * (1 - p.kt_brake)) * x.tq_tot;

% tq_front    = softplus(x.tq_tot) * p.kt_drive     	+ softminus(x.tq_tot) * p.kt_brake;
% tq_rear     = softplus(x.tq_tot) * (1 - p.kt_drive)	+ softminus(x.tq_tot) * (1 - p.kt_brake);
tq_front    = p.tq_gain * (x.tq_tot) * p.kt_drive     	;
tq_rear     = p.tq_gain * (x.tq_tot) * (1 - p.kt_drive)	;


% Simple (open) differential
tq_fl   = tq_front / 2;
tq_fr   = tq_front / 2;
tq_rl   = tq_rear / 2;
tq_rr   = tq_rear / 2;

% -------------------------------------------------------------------------
% ### CONTROLS
% -------------------------------------------------------------------------
sh.setODE('del_f',	1 / dsdt * u.ddel_f);
sh.setODE('tq_tot', 1 / dsdt * u.dtq_tot);

% -------------------------------------------------------------------------
% ### PLANAR CHASSIS DYNAMICS
% -------------------------------------------------------------------------
% Compute the final chassis planar accelerations
sh.setODE('vx',     1 / dsdt * ((fx - fd) / p.m + x.vy * x.dpsi));
sh.setODE('vy',     1 / dsdt * (fy / p.m - x.vx * x.dpsi));
sh.setODE('dpsi',   1 / dsdt * (mz / p.Izz));

% Path deviation
sh.setODE('epsi',   1 / dsdt * (x.dpsi - u.Cs * dsdt));
sh.setODE('ey',     1 / dsdt * (x.vy * cos(x.epsi) + x.vx * sin(x.epsi)) );

% -------------------------------------------------------------------------
% ### WHEEL SPIN DYNAMICS
% -------------------------------------------------------------------------
% Compute the rotational acceleration
sh.setODE('wfl',    1 / dsdt * ((tq_fl - p.re * fxt(1, :)) / p.Iw));
sh.setODE('wfr',    1 / dsdt * ((tq_fr - p.re * fxt(2, :)) / p.Iw));
sh.setODE('wrl',    1 / dsdt * ((tq_rl - p.re * fxt(3, :)) / p.Iw));
sh.setODE('wrr',    1 / dsdt * ((tq_rr - p.re * fxt(4, :)) / p.Iw));

% Update distance
% sh.setODE('dist',   1);
% sh.setODE('time',   1 / dsdt);
end

function pathcosts(ch,x,q,u,r)
dsdt      = (x.vx * cos(x.epsi) - x.vy * sin(x.epsi)) / (1 - x.ey .* u.Cs);
ch.add((1 / dsdt)^2);
end


function gridconstraints(ch,~,~,x,p)
% torque constraint
% ch.add(x.tq_tot, '<=', 1000);
%
%   % force constraint
%   ch.add(x.Fx^2+x.Fy^2, '<=', p.Fmax^2);
%
%   % road bounds
%   y_center = sin(x.x);
%   y_max = y_center + 0.5*p.road_bound;
%   y_min = y_center - 0.5*p.road_bound;
%   ch.add(x.y,'<=',y_max);
%   ch.add(x.y,'>=',y_min);

% ch.add(x.ey .* u.Cs, '<', 0.9);
ch.add(abs(x.vy / x.vx), '<=', 0.3);

end

% function animate(time,solution,x_road,y_center,y_min,y_max)
% 
% global testRun
% isTestRun = testRun;
% 
% ts = time(2)-time(1);
% x_car = solution.states.x.value;
% y_car = solution.states.y.value;
% 
% %% Initialize animation
% figure('units','normalized');hold on;grid on;
% car = plot(x_car(1),y_car(1),'Marker','pentagram','MarkerEdgeColor','k','MarkerFaceColor','y','MarkerSize',15);
% carLine = plot(x_car(1),y_car(1),'Color','b','LineWidth',3);
% plot(x_road,y_center,'Color','k','LineWidth',1,'LineStyle','--');
% plot(x_road,y_min   ,'Color','k','LineWidth',2.0,'LineStyle','-');
% plot(x_road,y_max   ,'Color','k','LineWidth',2.0,'LineStyle','-');
% legend('car','car trajectory');
% axis equal;xlabel('x[m]');ylabel('y[m]');
% pause(ts)
% 
% snap_at = floor(linspace(2,length(time),4));
% 
% %%
% for k = 2:1:length(time)
%     set(carLine, 'XData' , x_car(1:k));
%     set(carLine, 'YData' , y_car(1:k));
%     set(car    , 'XData' , x_car(k));
%     set(car    , 'YData' , y_car(k));
%     
%     if isempty(isTestRun) || (isTestRun==false)
%         pause(ts);
%     end
%     
%     % record image for docs
%     if k == snap_at(1)
%         snapnow;
%         snap_at = snap_at(2:end);
%     end
% end
% end
