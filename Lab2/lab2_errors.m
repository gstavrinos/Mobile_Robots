first_part = 'lab2_run';
last_part = '.mat';
x = zeros(5,1);
y = zeros(5,1);
for i=1:5
    filename = strcat(strcat(first_part, num2str(i)),last_part);
    load(filename);
    x(i) = cell2mat(datahistory(end,2));
    y(i) = cell2mat(datahistory(end,3));
end

%When the start point is -4,0
%the end point should be -4,-1
ep = [-4, -1];
mean_distance_from_end_point = 0;
for i = 1:5
    mean_distance_from_end_point = mean_distance_from_end_point + sqrt((ep(1)-x(i))^2+(ep(2)-y(i))^2);
end
mean_distance_from_end_point = mean_distance_from_end_point / i


d = zeros(5,1);
mean_distance_between_end_points = 0;
for i = 1:5
    for j = 1:5
        if i ~= j
            d(i) = d(i) + sqrt((x(j)-x(i))^2+(y(j)-y(i))^2);
        end
    end
    d(i) = d(i) / (j-1); %mean distance of point i from all the other 4
   mean_distance_between_end_points = mean_distance_between_end_points + d(i);
end
mean_distance_between_end_points = mean_distance_between_end_points / i