% NDT/RTK position recognizer
gnss = readmatrix('sunrise-20220917092642_L1C0G1_route_1-2_gnss.csv');
numSV = gnss(:,3);
fixType = gnss(:,4);
pDOP = gnss(:,5);
latitude = gnss(:,6);
longtitude = gnss(:,7);
altitude = gnss(:,8);

ndt = readmatrix('sunrise-20220917092642_L1C0G1_route_1-1_ndt.csv');
ee_building = [22.99665875 120.222584889 98.211];
x = ndt(:,3);
y = ndt(:,4);
z = ndt(:,5);
ndt_lla = enu2lla([x y z], ee_building, 'ellipsoid');
ndt_latitude = ndt_lla(:,1);
ndt_longtitude = ndt_lla(:,2);
ndt_altitude = ndt_lla(:,3);

%% Plot on matlab
% plot RTK position
for i = 1:length(numSV)
    if(fixType(i)~=4)
       rtk_pose = scatter3(latitude(i,1), longtitude(i,1), altitude(i,1), 'r.');
       %textscatter3(latitude(i,1), longtitude(i,1), altitude(i,1), num2str(numSV))
       hold on
    else
       rtk_pose = scatter3(latitude(i,1), longtitude(i,1), altitude(i,1), 'g.');
       %text(latitude(i,1), longtitude(i,1), altitude(i,1), num2str(numSV))
       hold on
    end
end

% plot NDT position
 for i = 1:length(ndt_latitude)
   ndt_pose = scatter3(ndt_latitude(i,1), ndt_longtitude(i,1), ndt_altitude(i,1), 'b.');
   hold on
 end

title('Position for hole 1')
xlabel('Latitude')
ylabel('Longtitude')
zlabel('Altitude')
legend([rtk_pose, ndt_pose], {'RTK', 'NDT'});

%% Export csv file
title = {'latitude' 'longtitude' 'altitude'};
rtk_lla = [title; num2cell(latitude) num2cell(longtitude) num2cell(altitude)];
ndt_lla = [title; num2cell(ndt_latitude) num2cell(ndt_longtitude) num2cell(ndt_altitude)];

writecell(rtk_lla, 'rtk_hole1.csv')
writecell(ndt_lla, 'ndt_hole1.csv')
