function [monoImageData] = loadImageData(framesFolder, imageRange)
% loadImageData Read captured image data into memory. Tries to load
% individual images or a saved mat file if one exists.


    %Read all images to memory
    imgDir = [framesFolder '/data'];
    fileNames = dir(fullfile(imgDir, '*.png'));
    
    imageNum = length(imageRange);
    
    %Get the width and height
    testIm = imread([imgDir '/' fileNames(1).name]);
    imHeight = size(testIm, 1);
    imWidth = size(testIm, 2);
    
    monoImageData = {};
    monoImageData.rectImages = zeros(imHeight, imWidth, imageNum);
    monoImageData.timestamps = zeros(1, imageNum); 

    % Read all image timestamps
    dateStrings = loadTimestamps(framesFolder);
    dateStrings = dateStrings(imageRange);
    timestamps = zeros(1, length(dateStrings));
    for i = 1:length(dateStrings)
        timestamps(i) =  datenum_to_unixtime(datenum(dateStrings(i)));
    end
    
    for k = 1:imageNum
            filename = fileNames(imageRange(k)).name;
            monoImageData.rectImages(:,:,k) = imread([imgDir '/' filename]);
            monoImageData.timestamps(k) = timestamps(k);
    end
    
    function dn = datenum_to_unixtime( date_num )
      dn =  (date_num - 719529)*86400;         %# 719529 == datenum(1970,1,1)
    end
end

