% simulate N trials of random sun locations

N = 10000;

 

sensorMax = 255; % 8 bit ADC reading corresponding to Imax (modify to reflect real HW)

sensorNoiseSigma = 5.0; % estimated sensor noise (gaussian)

%sensor reading range: 0 (no light) to sensorMax (brightest sunshine)

 

%TODO: model sensor noise and bias taking into account real hardware setup

sensorBiasSigma = 10.0;

% 6 random biases that are the same sensor biases for all N samples from each sensor

sensorBiasVector = sensorBiasSigma * randn(1,6);

% or hardcode the 6 biases here:

%sensorBiasVector = [0 0 0 0 0 0]

 

% normal vectors for all 6 sensors:

css_normals = [1 -1 0  0 0 0;

               0  0 1 -1 0 0;

               0  0 0  0 1 -1];

           

%columns 1, 3, 5 are the primary normal vectors           

primary_css_normals = css_normals(:,1:2:end);

 

% storage for each trial's vector error

errorsDeg = zeros(N,1);

errorSum = [0; 0; 0];

 

for ii=1:N

    % randomly choose a true sun vector location for each of N trials

    trueSunVector = 2*rand(3,1) - 1;

    trueSunVector = trueSunVector / norm(trueSunVector);

    

    % compute the true sensor readings (without noise or bias)

    sensorTrue = sensorMax * (trueSunVector') * css_normals;

    backInd = find(sensorTrue<0);

    sensorTrue(backInd) = 0; % zero out the readings from "backside" sensors

    

    %new sensor noises for each sample from each sensor

    sensorNoiseVector = sensorNoiseSigma * randn(1,6);

    

    % simulate the 6 sensor measurements for this trial as truth+noise+bias

    sensorMeasureVector = sensorTrue + sensorNoiseVector + sensorBiasVector;

    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % compute the sun vector (replace this code with function being

    % tested)...VT or UVA code...
    %{
    %sort sensor values in ascending order (second row is for sensor ID)
    %sortedMeasure(2,6);
    sortedMeasure(1,:) = sort(sensorMeasureVector);
    
    for j=1:6
        for k=1:6
            if sortedMeasure(1,k) == sensorMeasureVector(j)
                sortedMeasure(2,k) = j;
                break;
            end
            
        end
    end
    
    for m=4:6
        if sortedMeasure(2,m) == 2
            computedSunVector(1) = sortedMeasure(1,m)/sqrt(sortedMeasure(1,5)^2+sortedMeasure(1,4)^2+sortedMeasure(1,3)^2);
        elseif sortedMeasure(2,m) == 4
            computedSunVector(1) = -sortedMeasure(1,m)/sqrt(sortedMeasure(1,5)^2+sortedMeasure(1,4)^2+sortedMeasure(1,3)^2);
        elseif sortedMeasure(2,m) == 3
            computedSunVector(2) = sortedMeasure(1,m)/sqrt(sortedMeasure(1,5)^2+sortedMeasure(1,4)^2+sortedMeasure(1,3)^2);
        elseif sortedMeasure(2,m) == 5
            computedSunVector(2) = -sortedMeasure(1,m)/sqrt(sortedMeasure(1,5)^2+sortedMeasure(1,4)^2+sortedMeasure(1,3)^2);
        elseif sortedMeasure(2,m) == 1
            computedSunVector(3) = sortedMeasure(1,m)/sqrt(sortedMeasure(1,5)^2+sortedMeasure(1,4)^2+sortedMeasure(1,3)^2);
        elseif sortedMeasure(2,m) == 6
            computedSunVector(3) = -sortedMeasure(1,m)/sqrt(sortedMeasure(1,5)^2+sortedMeasure(1,4)^2+sortedMeasure(1,3)^2);
        end
    end
%}
    computedSunVector = (1/sensorMax)*inv(primary_css_normals)*[sensorMeasureVector(1)-sensorMeasureVector(2);

       sensorMeasureVector(3)-sensorMeasureVector(4);

       sensorMeasureVector(5)-sensorMeasureVector(6)];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    

    % error is: computed - true

    errorVector = computedSunVector-trueSunVector;

    % add up all the errors from each trail to compute stats later

    errorSum = errorSum+ errorVector;

    

    % compute the error in degrees between the trueSunVector and the

    % computedSunVector using the law of cosines

    errorAngleCos = dot(computedSunVector,trueSunVector)/norm(computedSunVector)/norm(trueSunVector);

    if errorAngleCos > 1.0 % account for small eps > 1.0 numerical

        errorAngleCos = 1.0;

    end

    errorAngleDeg = 180.0/pi*acos(errorAngleCos);

    

    % save the error from this trial

    errorsDeg(ii) = errorAngleDeg;

end

 

 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Display Performance:

 

% average error over all N trials

meanErrorVector = errorSum./N;

meanErrorMag = norm(meanErrorVector);

meanErrorDeg = 180.0/pi*asin(meanErrorMag);

 

% error standard deviation over all N trials

errorStdDev = std(errorsDeg);