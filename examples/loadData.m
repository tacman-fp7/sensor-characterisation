nSet = 8;

while(true)
expDir = 'C:\software\sensor-characterisation\examples\data';
cd(expDir);
%cd(sprintf('fingerData_%05d', nSet));

%fingerData = dlmread('fingerData/data.log', ' '); 
ft = dlmread(sprintf('set%02d/ft/data.log', nSet), ' ');
omegaPosition = dlmread(sprintf('set%02d/OmegaPosition/data.log', nSet), ' ');

ftResampled = interp1(ft(:, 2), ft(:, 3:end), omegaPosition(:, 2));

figure(1)

plot(ft(:,5))

pause(5)
end
%%
ftResampled = interp1(ft(:, 2), ft(:,3:end), fingerData(:,2)); 
omegaPositionResampled = interp1(omegaPosition(:,2), omegaPosition(:,3:end), fingerData(:,2));

figure(1);
plot(ft(:, 3:6));
title('FT');
figure(2);
plot(omegaPosition(:,3:end));
plot(bsxfun(@minus, omegaPosition(:, 3:end), omegaPosition(1, 3:end)));
title('Omega');
figure(3);
%plot(fingerData(:, 4));
%plot([bsxfun(@minus, fingerData(:, 3:end), fingerData(1, 3:end))]);
%title('Finger');

figure(1);

cd('..');

