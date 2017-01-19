function [ inlierPointsLeft, inlierPointsRight ] = extractBinnedFeatures( viLeftImage, viRightImage )

inlierPointsLeft = [];
inlierPointsRight = [];
       % Binning
       
        uBin = 1:floor(size(viLeftImage,2)/6):size(viLeftImage,2);
        vBin = 1:floor(size(viLeftImage,1)/2):size(viLeftImage,1);
        uBinSize = diff([uBin,size(viLeftImage,2)]);
        vBinSize = diff([vBin,size(viLeftImage,1)]);
        [UBIN, VBIN] = meshgrid(uBin, vBin);
        [UBINSIZE, VBINSIZE] = meshgrid(uBinSize, vBinSize);
        UBIN = UBIN(:);
        VBIN = VBIN(:);
        UBINSIZE = UBINSIZE(:);
        VBINSIZE = VBINSIZE(:);
        
        for b = 1:size(UBIN,1)
            roiVec = [UBIN(b), VBIN(b), UBINSIZE(b), VBINSIZE(b)];
            
            %Detect strongest corners
            leftPoints = detectSURFFeatures(viLeftImage,'ROI',roiVec);
            rightPoints = detectSURFFeatures(viRightImage,'ROI',roiVec);

            %leftPoints = leftPoints.selectStrongest(50);
            %rightPoints = rightPoints.selectStrongest(50);

            %Extract features and stereo match
           [featuresLeft, validLeftPoints] = extractFeatures(viLeftImage, leftPoints);
           [featuresRight, validRightPoints] = extractFeatures(viRightImage, rightPoints);

            indexPairs = matchFeatures(featuresLeft, featuresRight);
            matchedPointsLeft = validLeftPoints(indexPairs(:, 1), :);
            matchedPointsRight = validRightPoints(indexPairs(:, 2), :);

            inliers = abs((matchedPointsLeft.Location(:, 2) - matchedPointsRight.Location(:, 2))) <= 1 & abs((matchedPointsLeft.Location(:, 1) - matchedPointsRight.Location(:, 1))) > 3;

            inlierPointsLeft = [inlierPointsLeft; matchedPointsLeft(inliers).Location];
            inlierPointsRight = [inlierPointsRight; matchedPointsRight(inliers).Location];
        end

end

