first_part = 'lab2_run';
last_part = '.mat';
for i=1:5
    filename = strcat(strcat(first_part, num2str(i)),last_part);
    load(filename);
    x = cell2mat(datahistory(:,2));
    y = cell2mat(datahistory(:,3));
end