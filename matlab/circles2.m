function circles2(times, y, tagpos, pos)

n = size(y, 2);

fig = figure;
set(gcf, 'Color','white');
axis equal;
axis([min(pos(1, :))-200 max(pos(1, :))+800 min(pos(2, :))-1400 max(pos(2, :))+1000]);
hold on;
al = animatedline(pos(1, 1), pos(2,1), 'Color', [0.96 0.4 0.4], 'LineWidth', 0.55, 'MaximumNumPoints', 1000);
plot(tagpos(1, :), tagpos(2, :), 'o', 'MarkerFaceColor', 'c');
h = viscircles(tagpos', y(:, 1), 'Color', 'c');
%p = viscircles([pos(1, 1), pos(2, 1)], 25, 'Color', 'b');
p = plot(pos(1, 1),pos(2, 1),'o','MarkerFaceColor','red');

%set(gca, 'nextplot','replacechildren', 'Visible','off');

pause

%# create AVI object
vidObj = VideoWriter('circles.avi');
vidObj.Quality = 85;
vidObj.FrameRate = 30;
open(vidObj);

x = onCleanup( @() { close(gcf); close(vidObj); } );

for i = 2:n
    delete(h);
    h = viscircles(tagpos', y(:, i), 'Color', 'c');
    %viscircles([pos(1, i), pos(2, i)], 10, 'Color', 'm');
    %delete(p);
    %p = viscircles([pos(1, i), pos(2, i)], 25, 'Color', 'b');
    
    addpoints(al, pos(1, i), pos(2, i));
    p.XData = pos(1, i);
    p.YData = pos(2, i);
    
    drawnow
    writeVideo(vidObj, getframe(fig));
end
close(gcf);

close(vidObj);