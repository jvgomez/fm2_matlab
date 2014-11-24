function ppath = trimpath (path, parts)

ppath = [path(:,1) ; 1];

for i = 2:parts-1
    step = round(i*size(path,2)/parts);
    ppath(:,i) = [path(:,step); step];
end

% for i=1:round(size(path,2)/parts):size(path,2)
%    ppath(:,count) = path(:,i);    
%    count = count+1;
% end

ppath(:,parts) = [path(:,end) ; size(path,2)];