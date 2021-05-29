function [times, dees, ids] = spoon2uwb2(uwbstruct)
% uwbstruct is the output.uwb from parse_spoondata
% output is the parameters time and y for uwb_filter and anchor ids

% convert struct to matrix
celldata = struct2cell(uwbstruct);
data = cell2mat(celldata);

data = squeeze(data);

% get anchor ids from data
ids = unique(data(2, :));

% own time for each measurement
times = data(1, :);
%times = times(1:size(ids, 2):end);

% initialize dees to correct size
dees = nan(size(ids, 2), size(times, 2));

% values 1 to num of anchors, to map ids to
vals = 1:size(ids, 2);

% map anchor ids to a number
map = containers.Map(ids, vals);

n = size(data, 2);
lastvals = 1:size(map.Count);

i = 1;
for a = 1:size(dees, 2)
    
    % end running if no more data
    if i >= size(data, 2)
        break
    end
    
    %while isnan(dees(map(data(2, i)), a))
    
    % test that value is not the same as previous (missing range)
    if a == 1 || ( a > 1 && ( (data(3, i)) ~= lastvals(map(data(2, i))) ))
        % copy the value to dees and lastvals
        dees(map(data(2, i)), i) = (data(3, i));
        lastvals(map(data(2, i))) = (data(3, i));
        %a = a+1;
    end
    
    i = i +1;
    if i >= size(data, 2)
        break
    end
    
end

t = any(~isnan(dees));
dees = dees(:, t);
times = times(:, t);

[times, I] = sort(times);
dees = dees(:, I);