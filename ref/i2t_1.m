% I2t current limit development
% JFDuval, Dephy, Inc. 09/14/2016

clc;
close all;
clear all;
format short eng

%dt_sample = 12e-3;
dt_compute = 100e-3;

% Test waveform:
% ==============
VAL0 = 0;
VAL1 = 50;
VAL2 = 150;
LEN1 = 100;
LEN2 = 300;
LEN3 = 325;
LEN4 = 350;
LEN5 = 500;

expLenSec = 51;
expLenSamples = int32(expLenSec/dt_compute);
currentDigital = zeros(1,expLenSamples);

%Steady up
inc = (VAL1-VAL0)/LEN1;
for i = 1:LEN1
  currentDigital(i+1) = currentDigital(i) + inc;
end

%Plateau
for i = LEN1:LEN2
  currentDigital(i) = currentDigital(i-1);
end

% Spike (up)
inc = (VAL2-VAL1)/(LEN3-LEN2);
for i = LEN2:LEN3
  currentDigital(i) = currentDigital(i-1)+inc;
end

%Back to plateau
inc = (VAL1-VAL2)/(LEN4-LEN3);
for i = LEN3:LEN4
  currentDigital(i) = currentDigital(i-1)+inc;
end

%Plateau
for i = LEN4:LEN5
  currentDigital(i) = currentDigital(i-1);
end

figure()
plot(currentDigital, 'b')
title('Digitized current')

%===================

% Compute square and integrate:
LEAK = 3000;
integral = 0;
leakyIntegral = 0;
integralLog = zeros(1,expLenSamples);
integralLogLeaky = zeros(1,expLenSamples);
squaredLog = zeros(1,expLenSamples);
for i = 1:expLenSamples
  squared = currentDigital(i).^2;
  squaredLog(i) = squared;
  
  integral = integral + squared;
  
  leakyIntegral = leakyIntegral + squared - LEAK;
  if(leakyIntegral < 0)
    leakyIntegral = 0;
  end
  integralLogLeaky(i) = leakyIntegral;
  
  integralLog(i) = integral;
end

figure()
plot(squaredLog, 'r')
title('Squared value')

figure()
plot(integralLog, 'b')
hold on
plot(integralLogLeaky, 'r')
title('Integral over time')
legend('No leak', 'Leaky')

%===================

%Table of squares:

squared = zeros(1,256);
for i = 1:256
  squared(i) = int32((i-1)^2);
end

%====================

SCALE = 128;  %I2C_SCALE_DOWN_SHIFT = 7
I2T_LEAK = 6104;
I2T_LIMIT = 762891;
dt = 0.1;

TIME_AT_LIMIT_CURR = 10;    %s
CURR_LIMIT = 15000;         %mA

Limit = (TIME_AT_LIMIT_CURR / dt) * ( (CURR_LIMIT/SCALE)^2 - I2T_LEAK )

CURR = 11000:1000:30000;
tmp = (CURR./SCALE).^2;
time = (Limit * dt) ./ ( tmp - I2T_LEAK );
figure()
plot(CURR, time)
title('Time before reaching the I2t limit')
xlabel('Current (mA)')
ylabel('Time (s)')