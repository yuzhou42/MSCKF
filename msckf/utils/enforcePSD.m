function matrixPSD = enforcePSD(matrix)
%强制协方差变为对称矩阵：主对角线元素取绝对值，非对角线元素对称元素取平均值

    if size(matrix,1) ~= size(matrix,2)
        error('Input matrix is not symmetric.');
    else
        matrixPSD = matrix;
        for r = 1:size(matrixPSD,1)
            for c = 1:size(matrixPSD,2)
                if r == c
                    matrixPSD(r,c) = abs(matrixPSD(r,c));
                else
                     offDiagElement = mean([matrixPSD(r,c),matrixPSD(c,r)]);
%                      offDiagElement = matrixPSD(r,c);
                     matrixPSD(c,r) = offDiagElement;
                     matrixPSD(r,c) = offDiagElement;
                end
            end
        end
    end
end