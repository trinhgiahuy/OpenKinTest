function data = parse_spoondata(fname, channels)
% Henri Nurminen 15.9.2014
if nargin<2 || isempty(channels)
    channels = {'tim', 'acc', 'gyr', 'mag', 'prs', 'uwb', 'wla', 'gps'};
end

fileid = fopen(fname);
if fileid==-1
    error(['File ' fname 'does not exist.']);
end

% Initialize all the data structures:

ch = {'tim', 'acc', 'gyr', 'mag', 'prs', 'uwb', 'wla', 'gps'};
choseninds = nan(size(channels));
for i = 1:numel(channels)
    ci = find(strcmp(channels{i}, ch));
    if isempty(ci)
        error('Chosen meas. channel not available.');
    else
        choseninds(i) = ci;
    end
end
ch_chosen = ch(choseninds);
nch = numel(ch); % number of data channels
tim = nan(1,0);
acc = nan(3,0);
gyr = nan(3,0);
mag = nan(3,0);
prs = nan(1,0);
uwbstruct = struct('time', nan, 'id', nan, 'd', nan);
uwb = repmat(uwbstruct, [1,0]);
wlanstruct = struct('time', nan, 'SSID',[],'BSSID',[],'rss',[],'freq',[]);
wlanstruct.SSID = {};
wla = repmat(wlanstruct, [1,0]);
gpsstruct = struct('time', nan, 'pos', nan(3,1), 'acc', nan, 'brg', nan, 'spd', nan);
gps = repmat(gpsstruct, [1,0]);

inds = nan(nch,0); %indices for reference track index & each meas. channel

while true
    du = fgetl(fileid);
    if du==-1
        break;
    end
    
    tch = du(3:5);
    if ~any(strcmp(tch, ch_chosen))
        tch = 'skip';
    else
        cs = strfind(du, ',');
    end
    
    switch tch
        case 'skip'
            continue;
        case 'tim'
            ttim = str2double(du(cs(2)+4:end));
            tim = cat(2, tim, ttim);
            time = ttim;
        case 'acc'
            tacc = nan(3,1);
            tacc(1) = str2double(du(cs(1)+3:cs(2)-1));
            tacc(2) = str2double(du(cs(2)+3:cs(3)-1));
            tacc(3) = str2double(du(cs(3)+3:cs(4)-1));
            acc = cat(2, acc, tacc);
            time = str2double(du(cs(4)+4:cs(5)-1));
        case 'gyr'
            tgyr = nan(3,1);
            tgyr(1) = str2double(du(cs(1)+3:cs(2)-1));
            tgyr(2) = str2double(du(cs(2)+3:cs(3)-1));
            tgyr(3) = str2double(du(cs(3)+3:cs(4)-1));
            gyr = cat(2, gyr, tgyr);
            time = str2double(du(cs(4)+4:cs(5)-1));
        case 'mag'
            tmag = nan(3,1);
            if numel(cs)>2
            tmag(1) = str2double(du(cs(1)+3:cs(2)-1));
            tmag(2) = str2double(du(cs(2)+3:cs(3)-1));
            tmag(3) = str2double(du(cs(3)+3:cs(4)-1));
            mag = cat(2, mag, tmag);
            time = str2double(du(cs(4)+4:cs(5)-1));
            end
        case 'prs'
            tprs = str2double(du(cs(1)+3:cs(2)-1));
            prs = cat(2, prs, tprs);
            time = str2double(du(cs(2)+4:cs(3)-1));
        case 'uwb'
            tuwb = uwbstruct;
            %tuwb.time = str2double(du(cs(3)+4:cs(4)-1));
            tuwb.time = str2double(du(cs(2)+4:cs(3)-1));
            %tuwb.id = str2double(du(cs(1)+3:cs(2)-1));
            tuwb.id = hex2dec(du(cs(1)+3:cs(2)-1));
            tuwb.d = str2double(du(cs(4)+3:cs(5)-1));
            uwb = cat(2, uwb, tuwb);
            time = tuwb.time
        case 'wla'
            twla = wlanstruct;
            twla.time = str2double(du(cs(1)+11:end));
            duu = fgetl(fileid);
            css = strfind(duu, ',');
            while ~strcmp(duu, 'END')
                tSSID = duu(1:css(1)-1);
                tBSSID = duu(css(1)+1:css(2)-1);
                tBSSID(strfind(tBSSID,':')) = [];
                tBSSID = hex2dec(tBSSID);
                trss = str2double(duu(css(2)+1:css(3)-1));
                tfreq = str2double(duu(css(3)+1:end));
                twla.SSID = cat(2, twla.SSID, tSSID);
                twla.BSSID = cat(2, twla.BSSID, tBSSID);
                twla.rss = cat(2, twla.rss, trss);
                twla.freq = cat(2, twla.freq, tfreq);
                duu = fgetl(fileid);
                css = strfind(duu, ',');
            end
            wla = cat(2, wla, twla);
            time = twla.time;
        case 'gps'
            tgps = gpsstruct;
            tgps.time = str2double(du(cs(4)+4:cs(5)-1));
            tgps.pos = [str2double(du(cs(2)+4:cs(3)-1)); ...
                str2double(du(cs(1)+4:cs(2)-1)); ...
                str2double(du(cs(3)+4:cs(4)-1))];
            tgps.acc = str2double(du(cs(5)+5:cs(6)-1));
            tgps.alt = str2double(du(cs(6)+5:cs(7)-1));
            tgps.brg = str2double(du(cs(7)+5:cs(8)-1));
            tgps.spd = str2double(du(cs(8)+5:cs(9)-1));
            gps = cat(2, gps, tgps);
            time = tgps.time;
    end
    chind = find(strcmp(tch, ch));
    if isempty(inds)
        inittime = time;
        inds = zeros(nch,1);
        inds(chind) = 1;
    else
        timeind = time-inittime+1;
        inds = cat(2, inds, repmat(inds(:,end), [1,timeind-size(inds,2)]));
        inds(chind,end) = inds(chind,end) + 1;
    end
end
fclose(fileid);

% Measurement resolution:
Fs = 100;
Fs_inertial = 100;
data = struct('inittime',inittime,  'Fs',Fs, 'Fs_inertial',Fs_inertial, ...
    'sensorinds', inds(2:5,[1:1000/Fs_inertial:end-1,end]), ...
    'tim', tim, 'acc', acc, 'gyr', gyr, 'mag', mag, 'prs', prs, ...
    'inds', inds([1,6:end],[1:1000/Fs:end-1,end]), ...
    'uwb', uwb, 'wla', wla, 'gps', gps);