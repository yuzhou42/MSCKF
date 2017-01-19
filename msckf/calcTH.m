function [T_H, Q_1] = calcTH(H_o)
%函数功能：QR分解H_o矩阵，并且剔除Q中的零行，以及R中对应的列
%返回值：
%      T_H：R矩阵
%      Q_1：Q矩阵
%输入值：
%      H_o：待分解矩阵

%CALCTH Calculates T_H matrix according to Mourikis 2007

%对H矩阵进行QR分解
[Q,R] = qr(H_o);

%Find all zero rows of R
%判断R矩阵是否某一行全为0
%例如：
%     |1 2 4|
% R = |0 0 0|
%     |3 5 6|
%则：all(R==0,2)=[0;1;0]
isZeroRow = all(R==0, 2);

%Extract relevant matrices
%H_o = Q * R
%             |R1|
%    = |Q1 Q2||0 |
%提取R矩阵中非零行的元素
T_H = R(~isZeroRow, :);
%提取Q矩阵中非零列的元素
Q_1 = Q(:, ~isZeroRow);

end

