function pointplot(test)
xy = [test(~isnan(test(:, 27)), 28), test(~isnan(test(:, 27)), 29)];
xy = xy(xy(:,1) < 20 & xy(:,2) < 20 & xy(:,2) > -20 & xy(:,1) > -20, :);
n = size(xy, 1);
%hold on;
%for i=1:n
%    plot(xy(i, 1),xy(i, 2),'or','MarkerSize',5,'MarkerFaceColor','r');
%    axis([min(xy(:, 1)) max(xy(:, 1)) min(xy(:, 2)) max(xy(:, 2))]);
%    pause(.001);
%end

[dummy I]=unique(xy,'rows','first');
xy=xy(sort(I), :);

fn = 41;

B = 1/fn*ones(fn,1);
xy = filter(B,1,xy);
xy = xy(fn:end, :);

%blo = fir1(6,0.01,chebwin(7,100));

%xy = filter(blo,1,xy);


%xy = medfilt1(xy, 3);

fig = figure();
axis([min(xy(:, 1)) max(xy(:, 1)) min(xy(:, 2)) max(xy(:, 2))]);
comet_speed(xy(:, 1), xy(:, 2));