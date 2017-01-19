function [ cellArray ] = removeCells(inputCellArray, deleteIdx)
%REMOVECELLS Deletes all the cells in the inputCellArray at the deleteIdx
%indices

cellArray = inputCellArray;
cellArray(deleteIdx) = [];
cellArray(cellfun(@isempty, cellArray)) = [];%去掉数组元素为空的数组

end

