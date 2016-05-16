%% Fresnel zone calculations

%D = 1; f = 2.4;
%r = 8.657*sqrt(D/f)

D = 10; f = 2.4;
r1 = 8.657*sqrt(D/f)
r1 = 17.31*sqrt(5*5/(f*10))
%%

D = 20; f = 2.4;
r2 = 8.657*sqrt(D/f)

D = 50; f = 2.4;
r3 = 8.657*sqrt(D/f)

disp(['Radius of first fresnel zone'])
disp(['r1 = ',num2str(r1),' m',' @10km'])
disp(['r2 = ',num2str(r2),'m',' @20km'])
disp(['r3 = ',num2str(r3),'m',' @50km'])
%%

D = 1:1:50
r = 8.657*sqrt(D/f)'

figure(1)
plot(D,r)
grid
xlabel('Distance (km)'); ylabel('Radius of fresnel zone (m)')
title('Title')