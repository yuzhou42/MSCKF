function [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, msckfState, camStateIndices)
%函数功能：计算重投影误差对MSCKF状态量的雅克比矩阵（MSCKF观测模型）
%
%返回值：
%      H_o_j：A_j'*H_x_j
%      A_j：为H_f_j的左乘零空间变换矩阵，
%           H_f_j是相机坐标系下齐次坐标(x y)对世界坐标系下3D点的雅克比矩阵           
%      H_x_j：得到相机坐标系下齐次坐标(x y)对msckf中状态量的雅克比矩阵
%输入值：
%      p_f_G：特征点在global坐标系下坐标
%      msckfState：当前msckf状态
%      camStateIndices：当前窗口中相机状态索引

%CALCHOJ Calculates H_o_j according to Mourikis 2007
% Inputs: p_f_G: feature location in the Global frame
%         msckfState: the current window of states
%         camStateIndex: i, with camState being the ith camera pose in the window       
% Outputs: H_o_j, A


N = length(msckfState.camStates);
M = length(camStateIndices);
H_f_j = zeros(2*M, 3);
H_x_j = zeros(2*M, 12 + 6*N);


c_i = 1;
for camStateIndex = camStateIndices
    camState = msckfState.camStates{camStateIndex};

    C_CG = quatToRotMat(camState.q_CG);
    %The feature position in the camera frame
    %得到3D点在当前相机坐标系下的坐标
    p_f_C = C_CG*(p_f_G - camState.p_C_G);

    X = p_f_C(1);
    Y = p_f_C(2);
    Z = p_f_C(3);

    %得到相机坐标系下齐次坐标对非齐次坐标的雅克比矩阵
    % x = X/Z y = Y/Z
    % |1/Z  0   -X/Z^2|
    % |0   1/Z  -Y/Z^2|
    J_i = (1/Z)*[1 0 -X/Z; 0 1 -Y/Z];

    %得到相机坐标系下齐次坐标(x y)对世界坐标系下3D点的雅克比矩阵
    H_f_j((2*c_i - 1):2*c_i, :) = J_i*C_CG;

    %注意：msckf中状态量是误差状态量，投影误差只和相机有关，因此只对相机姿态和位置求导的雅克比不为0
    %得到相机坐标系下齐次坐标(x y)对相机姿态的雅克比矩阵
    H_x_j((2*c_i - 1):2*c_i,12+6*(camStateIndex-1) + 1:12+6*(camStateIndex-1) + 3) = J_i*crossMat(p_f_C);
    %得到相机坐标系下齐次坐标(x y)对相机位置的雅克比矩阵
    H_x_j((2*c_i - 1):2*c_i,(12+6*(camStateIndex-1) + 4):(12+6*(camStateIndex-1) + 6)) = -J_i*C_CG;

    c_i = c_i + 1;
end

%参考文献：“The Battle for Filter Supremacy: A Comparative Study of the
%      Multi-State Constraint Kalman Filter and the Sliding Window Filter”
%公式47
%error = z - z_hat = H_x_j * x + H_f_j * p + R_j(x是msckf中的状态，p是3D点，R_j为噪声)
% 左右同乘H_f_j的零空间变换矩阵A_j：
% ==> A_j' * error = A_j' * H_x_j * x + A_j' * H_f_j * p + A_j' * R_j * A_j
% ==> A_j' * error = A_j' * H_x_j * x + 0 + A_j' * R_j * A_j
A_j = null(H_f_j');
H_o_j = A_j'*H_x_j;

end

