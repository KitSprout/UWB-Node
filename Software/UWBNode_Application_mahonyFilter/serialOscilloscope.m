clear;

dataLens = 15;
dataType = 'single';

s = kSerial(115200, 'clear');
s.dataBuffer = zeros(dataLens, 1024 * 16);
s.open();

fig = figure(1);
set(fig, 'Position', [100, 140, 1200, 600], 'color', 'w');

figSub = subplot(1, 1, 1);
osc = kOscilloscope();
osc.setWindow(20, -20, 1000);
osc.curveChannel = [6, 7, 8];
osc.curveColor   = ['r', 'g', 'b'];
osc.curveScale   = [1, 1, 1];
osc.initOscilloscope(figSub);

while ishandle(osc.fig)
    [packetData, packetLens] = s.packetRecv(dataLens, dataType);
    if packetLens > 0
        s.dataBuffer = [s.dataBuffer(:, packetLens + 1 : end), packetData];       % record data
        time = s.dataBuffer( 1 : 2, end);
        gyr  = s.dataBuffer( 3 : 5, end);
        acc  = s.dataBuffer( 6 : 8, end);
        mag  = s.dataBuffer( 9 : 11, end);
        q    = s.dataBuffer(12 : 15, end);

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
Cq = [ 2 * (q0*q0 + q1*q1) - 1, 2 * (q1*q2 - q0*q3),     2 * (q1*q3 + q0*q2); ...
       2 * (q1*q2 + q0*q3),     2 * (q0*q0 + q2*q2) - 1, 2 * (q2*q3 - q0*q1); ...
       2 * (q1*q3 - q0*q2),     2 * (q2*q3 + q0*q1),     2 * (q0*q0 + q3*q3) - 1 ];
att = [ asin(-Cq(1, 3)); atan2(-Cq(2, 3), Cq(3, 3)); atan2(-Cq(1, 2), Cq(1, 1)) ] * 180 / pi;

        freq = fix(s.getFreq([1, 2], 100));
        tt = [fix(time(1) / 60); rem(fix(time(1)), 60); fix(time(2) * 100 + 1e-5)];
        seqNum = s.packet.sequenceNum;
        osc.updateOscilloscope(s);
        fprintf('[%05i][%02i][%02i:%02i:%02i][%3dHz] Gyr[%9.3f, %9.3f, %9.3f] Acc[%6.3f, %6.3f, %6.3f] Mag[%8.3f, %8.3f, %8.3f] Att[%7.3f, %8.3f, %8.3f]\n', seqNum, packetLens, tt, freq, gyr, acc, mag, att);
    end
end

s.close();
s.save2mat('imuData', {'sec(1)', 'msec(2)', 'gyr(3:5)', 'acc(6:8)', 'mag(9:11)', 'att(12:14)', 'q(15:18)'});
