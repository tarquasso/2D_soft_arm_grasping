function [ low,high ] = splitUp( val )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
low = uint8(uint16(192) + bitand(val, uint16(31)));
high = uint8(bitand(bitshift(val, -5), uint16(127)));
end

