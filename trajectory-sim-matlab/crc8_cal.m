function crc = crc8_cal(data)
    crc = uint8(0);
    for i = 1:length(data)
        crc = bitxor(crc, uint8(data(i)));
        for j = 1:8
            if bitand(crc, uint8(128))
                crc = bitxor(bitshift(crc, 1), uint8(7));
            else
                crc = bitshift(crc, 1);
            end
        end
    end
end
