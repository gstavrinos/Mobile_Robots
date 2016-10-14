first_part = 'lab2_run';
last_part = '.mat';
visualize_test_path = true; %set to true to visualize the test run
for i=1:5
    filename = strcat(strcat(first_part, num2str(i)),last_part);
    load(filename);
    x = cell2mat(datahistory(:,2));
    y = cell2mat(datahistory(:,3));
    figure
    %plot(x,y)
    plot(x,y,'.')
end

if visualize_test_path
    filename = 'lab2_test_run.mat';
    load(filename);
    x = cell2mat(datahistory(:,2));
    y = cell2mat(datahistory(:,3));
    figure
    %plot(x,y)
    plot(x,y,'.')
end;