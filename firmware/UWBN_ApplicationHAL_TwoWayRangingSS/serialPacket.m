clear;

s = kSerial(115200, 'clear');
s.setRecordBufferSize(1024 * 16);
s.setRecvThreshold(0);
s.open();

for i = 1 : 100000
    [packetData, packetInfo, packetLens] = s.packetRecv();
    if ~isempty(packetLens) && packetLens > 0
        gyr  = s.ks.data( 1:  3, end) / 10;
        acc  = s.ks.data( 4:  6, end);
        mag  = s.ks.data( 7:  9, end) / 10;
        tim  = s.ks.data(10: 11, end);
        tt   = s.getTime([10, 11], 0, 0.001);
        freq = s.getFreq([10, 11], 50, 0.001);

        fprintf('[%06i][%02i]', s.ks.lens, packetLens);
        fprintf('[%02i:%02i:%02i][%4.0fHz] ', tt(1), tt(2), fix(tt(3) / 10), freq);
        fprintf('Gyr[%6.0f, %6.0f, %6.0f] Acc[%6.0f, %6.0f, %6.0f] Mag[%6.1f, %6.1f, %6.1f] ', gyr, acc, mag);
        if size(s.ks.data, 1) == 18
            ang  = s.ks.data(12: 14, end) / 10;
            sgy  = s.ks.data(15: 17, end) / 10;
            bmg  = s.ks.data(18, end) / 100;
            fprintf('Ang[%6.1f, %6.1f, %6.1f] Sgy[%6.1f, %6.1f, %6.1f] Bmg[%6.1f] ', ang, sgy, bmg);
        end
        fprintf('\n');
    end
end
s.close();

% {
% check packet
tt = s.record.data(10, end - s.record.count + 1 : end) + s.record.data(11, end - s.record.count + 1 : end) / 1000;
dt = tt(2 : end) - tt(1 : end - 1);
% plot(1 : s.record.count - 1, dt);
res = find(dt > 1 / freq + 1e-10 );
if isempty(res)
    fprintf('---- No packet loss ----\n');
else
    fprintf('---- Packet loss - %i ----\n', size(res, 2));
end
%}
%{
s.save2mat('rawData', {'gyr(1:3)', 'acc(4:6)', 'mag(7:9)', 'sec(10)', 'msc(11)', 'ang(12:14)', 'sgy(15:17)', 'bmg(18)'});
%}