close all;
clearvars -except X

figure('name',"Y-X");
plot(X(1,:),X(2,:),'b')
hold on;


%Simulate:
dt = 0.1;
V = X(4,:);
W = X(5,:);

%Simulation step=0.1
xn(1)=0;
yn(1)=0;
titan(1)=0;
for i=2:1:length(X)     
    xn(i) = xn(i-1) +dt*cos(titan(i-1))*V(i-1);
    yn(i) = yn(i-1) +dt*sin(titan(i-1))*V(i-1);
    titan(i) = titan(i-1) +dt*W(i-1);
    
    plot(xn,yn,'--g');
end


%Simulation step=0.01
x(1)=0;
y(1)=0;
tita(1)=0;

i0 = 2;
for j=2:1:length(X)
    for i=i0:1:(i0+10)        
        x(i) = x(i-1) +dt*0.1*cos(tita(i-1))*V(j-1);
        y(i) = y(i-1) +dt*0.1*sin(tita(i-1))*V(j-1);
        tita(i) = tita(i-1) +dt*0.1*W(j-1);
        
        plot(x,y,'-.r');
    end
    i0=i0+10;
end
