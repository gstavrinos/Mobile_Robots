function showAll()
    allfiles = dir('mat_files');
    i = 0;
    for file = allfiles'
        i = i + 1;
        if i ~= 1 && i ~= 2
            load(strcat('mat_files/',file.name))
            figure('name',file.name)
            plot(cell2mat(datahistory(:,2)),cell2mat(datahistory(:,3)));
            title(file.name);
        end
    end