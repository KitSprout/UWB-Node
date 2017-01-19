clear;

dataLens = 18;
dataType = 'single';

s = kSerial(115200, 'clear');
s.dataBuffer = zeros(dataLens, 1024 * 16);
s.open();

getFirstSequenceNum = true;
firstSequenceNum = 0;

tic
for i = 1 : 10000
%while true
    [packetData, packetLens] = s.packetRecv(dataLens, dataType);
    if packetLens > 0
        s.dataBuffer = [s.dataBuffer(:, packetLens + 1 : end), packetData];       % record data
        time = s.dataBuffer( 1 : 2, end);
        gyr  = s.dataBuffer( 3 : 5, end);
        acc  = s.dataBuffer( 6 : 8, end);
        mag  = s.dataBuffer( 9 : 11, end);
        att  = s.dataBuffer(12 : 14, end);
        q    = s.dataBuffer(15 : 18, end);

        freq = fix(s.getFreq([1, 2], 100));
        tt = [fix(time(1) / 60); rem(fix(time(1)), 60); fix(time(2) * 100 + 1e-5)];
        seqNum = s.packet.sequenceNum;
        fprintf('[%05i][%02i][%02i:%02i:%02i][%3dHz] Gyr[%9.3f, %9.3f, %9.3f] Acc[%6.3f, %6.3f, %6.3f] Mag[%8.3f, %8.3f, %8.3f] Att[%7.3f, %8.3f, %8.3f]\n', seqNum, packetLens, tt, freq, gyr, acc, mag, att);
        if getFirstSequenceNum
            firstSequenceNum    = seqNum;
            getFirstSequenceNum = false;
        end
    end
end

lostPacket = (s.packet.sequenceNum - firstSequenceNum + 1) - s.packet.packetCount;
sampleRate = s.packet.packetCount / toc;
fprintf('\nrecv packet = %d, lost packet = %d, sample rate = %.3f Hz\n', s.packet.packetCount, lostPacket, sampleRate);

s.close();
s.save2mat('imuData', {'sec(1)', 'msec(2)', 'gyr(3:5)', 'acc(6:8)', 'mag(9:11)', 'att(12:14)', 'q(15:18)'});
