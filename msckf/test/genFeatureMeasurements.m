function pixelMeasurements = genFeatureMeasurements(T_wCam_GT, landmarks_w, K, simSetup)
%Will return -1 if the feature cannot be seen

camRes = simSetup.cameraResolution;

pixelMeasurements = zeros(2, size(T_wCam_GT, 3));

% Extract all viewable measurements
for step_i = 1:size(T_wCam_GT,3)
        
        %Transform and project landmarks into the camera frame
        landmarks_cam = homo2cart(inv(T_wCam_GT(:,:,step_i))*cart2homo(landmarks_w));
        pixels = homo2cart(K*landmarks_cam);
        
        %Determine which landmarks are viewable
        isInFieldOfView = pixels(1,:) > 0 & pixels(1,:) < camRes(2) & pixels(2,:) > 0 & pixels(2,:) < camRes(1);
        isInFieldOfView = isInFieldOfView & landmarks_cam(3,:) > 0;
        
        
        %viewableLandmarkIds = find(viewableLandmarksIdx);
        %viewableLandmarkIds = viewableLandmarkIds(:);
        %viewableLandmarks = landmarks_cam(:, viewableLandmarksIdx);
        if ~isInFieldOfView
            pixels = [-1; -1];
            n_p = [0; 0];
        else
            n_p = simSetup.pixelNoiseStd*randn(2,size(pixels,2)); %image noise        
        end
        
        pixelMeasurements(:, step_i) =  pixels + n_p;
        
end


end

