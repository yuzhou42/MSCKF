function [p_f_G, Jnew, RCOND] = calcGNPosEst(camStates, observations, noiseParams)
%函数功能：使用逆深度参数构造重投影误差函数，用高斯牛顿优化的方法估计特征点3D坐标
%
%返回值：
%      p_f_G：global坐标系下3x1特征向量
%      Jnew：chi2指标，表示残差大小
%      RCOND：估算1-条件数的倒数
%输入值：
%      camStates：M个相机状态
%      observations：当前路标点的2XM像素坐标矩阵
%      noiseParams： 图像像素误差

%CALCGNPOSEST Calculate the position estimate of the feature using Gauss
%Newton optimization
%   INPUT:
%   observations: 2xM matrix of pixel values of the current landmark
%   camStates: Cell array of M structs of camera poses
%   camera: intrinsic calibration
%   OUTPUT:
%   p_f_G: 3x1 feature vector in the global frame

%K is not needed if we assume observations are not pixels but x' = (u -
%c_u)/f_u

%K = [camera.f_u 0 camera.c_u; 0 camera.f_v camera.c_v; 0 0 1];

%Get initial estimate through intersection
%Use the first 2 camStates
%选择第一帧相机（camStates{1}）和最后一帧相机（camStates{secondViewIdx}）的特征点三角化恢复3D点
%获取相机状态长度
secondViewIdx = length(camStates);
%获得第一帧相机与最后一帧之间的位姿（2到1）
C_12 = quatToRotMat(camStates{1}.q_CG)*quatToRotMat(camStates{secondViewIdx}.q_CG)';
t_21_1 = quatToRotMat(camStates{1}.q_CG)*(camStates{secondViewIdx}.p_C_G - camStates{1}.p_C_G);
%三角化恢复3D点
p_f1_1_bar = triangulate(observations(:,1), observations(:,secondViewIdx),C_12, t_21_1);

%initialEst = quatToRotMat(camStates{1}.q_CG)'*p_f1_1_bar + camStates{1}.p_C_G;


%参考文献1：“The Battle for Filter Supremacy: A Comparative Study of the
%     Multi-State Constraint Kalman Filter and the Sliding Window Filter”

%参考文献2：“A Multi-State Constraint Kalman Filter
%     for Vision-aided Inertial Navigation”


xBar = p_f1_1_bar(1);
yBar = p_f1_1_bar(2);
zBar = p_f1_1_bar(3);
%用逆深度参数化3D点
alphaBar = xBar/zBar;
betaBar = yBar/zBar;
rhoBar = 1/zBar;

%xEst为逆深度形式表示的参数向量
xEst = [alphaBar; betaBar; rhoBar];
%获取相机状态长度
Cnum = length(camStates);

%Optimize
%设置高斯牛顿优化的参数（迭代次数）
maxIter = 10;
Jprev = Inf;

for optI = 1:maxIter
    %初始化误差向量E和误差权重矩阵W
    E = zeros(2*Cnum, 3);
    W = zeros(2*Cnum, 2*Cnum);
    errorVec = zeros(2*Cnum, 1);

    for iState = 1:Cnum
        %Form the weight matrix
        W((2*iState - 1):(2*iState),(2*iState - 1):(2*iState)) = diag([noiseParams.u_var_prime noiseParams.v_var_prime]);

        C_i1 = quatToRotMat(camStates{iState}.q_CG)*(quatToRotMat(camStates{1}.q_CG)');
        t_1i_i = quatToRotMat(camStates{iState}.q_CG)*(camStates{1}.p_C_G - camStates{iState}.p_C_G);
        

        %Form the error vector
        zHat = observations(:, iState);
        %参考文献1：公式36
        %参考文献2：公式32-公式37
        % |h1|         |alpha|
        % |h2| = Ci1 * |beta | + rho * t_1i_i
        % |h3|         |  1  |
        % 为了推导方便，这么表达：
        % |h1|                             |alpha|
        % |h2| =(Ci1(:1),Ci1(:1),Ci1(:1))* |beta | + rho * t_1i_i
        % |h3|                             |  1  |
        h = C_i1*[alphaBar; betaBar; 1] + rhoBar*t_1i_i;

        %计算重投影误差
        %参考文献1：公式37
        %e = z - |h1/h3|
        %        |h2/h3|
        errorVec((2*iState - 1):(2*iState),1) = zHat - [h(1); h(2)]/h(3);

        %Form the Jacobian
        %计算重投影误差对逆深度参数xEst的雅克比矩阵
        %参考文献1：公式39
        %de/dh = |-1/h3     0      h1/h3^2|
        %        |0       -1/h3    h2/h3^2|
        %dh/d(alpha,beta,rho) = |C_i1(:,1)  C_i1(:,2)  t_li_i|
        
        %de/d(alpha,beta,rho) = (de/dh) * (dh/d(alpha,beta,rho))
        %                     = |-1/h3     0      h1/h3^2| * |C_i1(:,1)  C_i1(:,2)  t_li_i|
        %                       |0       -1/h3    h2/h3^2|
        %                                                    |C_i1(1,1)  C_i1(1,2)  t_li_i(1)|
        %                     = |-1/h3     0      h1/h3^2| * |C_i1(2,1)  C_i1(2,2)  t_li_i(2)|
        %                       |0       -1/h3    h2/h3^2|   |C_i1(3,1)  C_i1(3,2)  t_li_i(3)|
        dEdalpha = [-C_i1(1,1)/h(3) + (h(1)/h(3)^2)*C_i1(3,1); ...
                    -C_i1(2,1)/h(3) + (h(2)/h(3)^2)*C_i1(3,1)];

        dEdbeta =  [-C_i1(1,2)/h(3) + (h(1)/h(3)^2)*C_i1(3,2); ...
                    -C_i1(2,2)/h(3) + (h(2)/h(3)^2)*C_i1(3,2)];

        dEdrho =   [-t_1i_i(1)/h(3) + (h(1)/h(3)^2)*t_1i_i(3); ...
                    -t_1i_i(2)/h(3) + (h(2)/h(3)^2)*t_1i_i(3)];

        Eblock = [dEdalpha dEdbeta dEdrho];
        %构造所有特征点的投影误差关于逆深度参数的雅克比矩阵
        E((2*iState - 1):(2*iState), :) = Eblock;
    end
    
    %Calculate the cost function
    %计算代价函数(用于判断终止条件)：0.5 * error' * W^(-1) * error 
    Jnew = 0.5*errorVec'*(W\errorVec);
    %Solve!
    %求解优化函数，（E'*W^(-1)*E）* dx_star = -E'*W^(-1)*errorVec，得到迭代逆参数xEst增量
    EWE = E'*(W\E);
    %RCOND返回值接近1结果优，返回接近0结果差
    RCOND = rcond(EWE);
    dx_star =  (EWE)\(-E'*(W\errorVec));
    
    xEst = xEst + dx_star;
    %计算迭代下降程度，用于判断终止条件，下降不动了，就认为收敛到最优了
    Jderiv = abs((Jnew - Jprev)/Jnew);
    
    Jprev = Jnew;

    if Jderiv < 0.01
        break;
    else
        alphaBar = xEst(1);
        betaBar = xEst(2);
        rhoBar = xEst(3);
    end
    
end

%将逆深度形式表示的3D点转换成欧式坐标系下的3D点坐标
p_f_G = (1/xEst(3))*quatToRotMat(camStates{1}.q_CG)'*[xEst(1:2); 1] + camStates{1}.p_C_G; 

        %三角化恢复3D点
    function [p_f1_1] = triangulate(obs1, obs2, C_12, t_21_1)
    %obs1，obs2：两个相机中匹配特征点坐标（相机坐标系）
    %C_12，t_21_1：两个相机之间的变换关系
        
        % triangulate Triangulates 3D points from two sets of feature vectors and a
        % a frame-to-frame transformation

           %Calculate unit vectors
           %将相机坐标系下坐标变为齐次坐标[x,y,1]
           v_1 = [obs1;1];
           v_2 = [obs2;1];
           %归一化坐标
           v_1 = v_1/norm(v_1);
           v_2 = v_2/norm(v_2);

%            P_f1
%            / \
% t_c1_P_f1 /   \ t_c2_P_f1
%          /     \
%        c1-------c2
%             t
%
% p_f1_1：表示3D点在P_f1在c1坐标系下坐标
% c1：表示第一个相机
% c2：表示第二个相机
% t_21_1：c1与c2之间的向量（c1指向c2，c1坐标系）
% v1：c1与3D点P_f1之间的单位向量（c1指向P_f1，c1坐标系），则：t_c1_P_f1 = v1 * scale1
% v2：c2与3D点P_f1之间的单位向量（c2指向P_f1，c2坐标系），则：t_c2_P_f1 = C_12 * （v2 * scale2）

% t_c1_P_f1 - t_c2_P_f1 = t
%                   |scale1|
% ===>[v1 -C_12*v_2]|scale2| = t_21_1
%
           A = [v_1 -C_12*v_2];
           b = t_21_1;

           scalar_consts = A\b;
           p_f1_1 = scalar_consts(1)*v_1;
    end

end

