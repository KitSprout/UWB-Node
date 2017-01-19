clear;

dataLens = 15;
dataType = 'single';

s = kSerial(115200, 'clear');
s.dataBuffer = zeros(dataLens, 1024 * 16);
s.open();

fig = figure(1);
set(fig, 'Position', [100, 140, 1200, 600], 'color', 'w');

figSub = subplot(1, 1, 1);
cube = kCube([0, 0, 0], [1.5, 1.5, 0.5], [1.5, 1.5, 1.5]);   % origin, scale, window
cube.initCube(figSub, [315, 30]);           % view

while ishandle(figSub)
    [packetData, packetLens] = s.packetRecv(dataLens, dataType);
    if packetLens > 0
        s.dataBuffer = [s.dataBuffer(:, packetLens + 1 : end), packetData];     % record data
        time = s.dataBuffer( 1 : 2, end);
        gyr  = s.dataBuffer( 3 : 5, end);
        acc  = s.dataBuffer( 6 : 8, end);
        mag  = s.dataBuffer( 9 : 11, end);
        q    = s.dataBuffer(12 : 15, end);
        att = [ asin(-cube.rotate(1, 3)); atan2(-cube.rotate(2, 3), cube.rotate(3, 3)); atan2(-cube.rotate(1, 2), cube.rotate(1, 1)) ] * 180 / pi;

        freq = fix(s.getFreq([1, 2], 100));
        tt = [fix(time(1) / 60); rem(fix(time(1)), 60); fix(time(2) * 100 + 1e-5)];
        seqNum = s.packet.sequenceNum;
        cube.qText.String = num2str([ tt(1), tt(2), tt(3), ...
                                      freq, ...
                                      gyr(1), gyr(2), gyr(3), ...
                                      acc(1), acc(2), acc(3), ...
                                      mag(1), mag(2), mag(3), ...
                                      att(1), att(2), att(3), ...
                                      q(1), q(2), q(3), q(4) ], ...
                                     'TIME [ %02i:%02i:%02i ]\n\nFreq = %6.2fHz\n\ngyr.x = %9.4f\ngyr.y = %9.4f\ngyr.z = %9.4f\n\nacc.x = %8.5f\nacc.y = %8.5f\nacc.z = %8.5f\n\nmag.x = %8.3f\nmag.y = %8.3f\nmag.z = %8.3f\n\natt.x = %9.4f\natt.y = %9.4f\natt.z = %9.4f\n\nq0 = %8.5f\nq1 = %8.5f\nq2 = %8.5f\nq3 = %8.5f');
        cube.plotCube([0, 0, 0], q);  % origin, quaternion
        fprintf('[%05i][%02i][%02i:%02i:%02i][%3dHz] Gyr[%9.3f, %9.3f, %9.3f] Acc[%6.3f, %6.3f, %6.3f] Mag[%8.3f, %8.3f, %8.3f] Att[%7.3f, %8.3f, %8.3f]\n', seqNum, packetLens, tt, freq, gyr, acc, mag, att);
    end
end

s.close();
s.save2mat('imuData', {'sec(1)', 'msec(2)', 'gyr(3:5)', 'acc(6:8)', 'mag(9:11)', 'att(12:14)', 'q(15:18)'});
