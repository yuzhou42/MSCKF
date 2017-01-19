function oxts = loadOxtsliteData(base_dir,frames)
% reads GPS/IMU data from files to memory. requires base directory
% (=sequence directory as parameter). if frames is not specified, loads all frames.

if nargin==1

  ts = loadTimestamps([base_dir '/oxts']);
  oxts  = [];
  for i=1:length(ts)
    if ~isempty(ts{i})
      oxts{i} = dlmread([base_dir '/oxts/data/' num2str(i-1,'%010d') '.txt']);
    else
      oxts{i} = [];
    end
  end
   
else

  k = 1;
  oxts = [];
  for i=1:length(frames)
    try
      file_name = [base_dir '/oxts/data/' num2str(frames(i)-1,'%010d') '.txt'];
      oxts{k} = dlmread(file_name);
    catch e
      oxts{k} = [];
    end
    k=k+1;
  end

end

