close all;
clearvars -except X
d = 3;
K1 = 1;
ang_d = 0;
K2 = 1E6;
K3 = 1E5;
yaw_d = -1.57;

ap = 0;
xp = 4;
yp = 0;

dist = sqrt( (xp-X(1,:)).^2 + (yp-X(2,:)).^2);
yaw = atan2(yp-X(2,:),xp-X(1,:));
ang = ap-X(3,:);
Coste_d = K1*(dist - d).^2;
Coste_ang = K2*((ang)-ang_d).^2;
Coste_yaw = K3*( yaw - yaw_d).^2;
Coste = Coste_d+Coste_ang+Coste_yaw;

figure('name','Costes');
subplot(2,2,1)
plot(Coste_yaw);
title('Coste_yaw')

subplot(2,2,2)

plot(Coste_d)
title('Coste_d')

subplot(2,2,3)
plot(Coste_ang)
title('Coste_ang')

subplot(2,2,4)
plot(Coste)
title('Coste total')

figure('name',"Y-X");
subplot(2,2,1)
plot(X(1,:),X(2,:),'b')
title("Y-X");
subplot(2,2,2)
plot(yaw)
title("Yaw")
subplot(2,2,3)
plot(ang)
title("ang")
subplot(2,2,4)
plot(dist)
title("dist")


%{ 
hold on;

Simulate:
dt = 0.1;
V = X(4,:);
W = X(5,:);


%Simulation step=0.1
xn(1)=0;
yn(1)=0;
titan(1)=0;
for i=2:1:length(X) 
    titan(i) = titan(i-1) +dt*W(i-1);
    xn(i) = xn(i-1) +dt*cos(titan(i))*V(i-1);
    yn(i) = yn(i-1) +dt*sin(titan(i))*V(i-1);
    
    plot(xn,yn,'--g');
end


%Simulation step=0.01
x(1)=0;
y(1)=0;
tita(1)=0;

i0 = 2;
for j=2:1:length(X)
    for i=i0:1:(i0+10)
        tita(i) = tita(i-1) +dt*0.1*W(j-1);
        x(i) = x(i-1) +dt*0.1*cos(tita(i))*V(j-1);
        y(i) = y(i-1) +dt*0.1*sin(tita(i))*V(j-1);
        
        plot(x,y,'-.r');
    end
    i0=i0+10;
end

%}

    

