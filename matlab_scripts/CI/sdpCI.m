function [c, C, omega] = sdpCI(a, A, b, B, H)
    % 输入:
    % a - 第一个估计值
    % A - 第一个估计值的协方差矩阵
    % b - 第二个估计值
    % B - 第二个估计值的协方差矩阵
    % H - 观测矩阵

    % 输出:
    % c - 融合后的估计值
    % C - 融合后的协方差矩阵
    % omega - 最优权重

    % CVX求解SDP
    d = length(A); %判断矩阵维度
    
    cvx_begin sdp
        % 变量定义
        variable omega(1) nonnegative
        variable Y(d,d) symmetric
        Ai = inv(A);
        Bi = inv(B);
        I = eye(d);
        
        minimize( trace(Y) ) % 目标函数，最小化trace(Y)


        % 半定性约束，C 是半正定的
        [Y, I; I, omega * Ai + (1 - omega) * H'* Bi * H] >= 0;
        omega >= 0;
        omega <= 1;
    cvx_end
    
    % C_inv = inv(Y);
    % C = inv(C_inv);
    C=Y;
    % 计算融合后的均值
    nu = b - H * a;
    W = (1 - omega) * C * H' * Bi;
    c = a + W * nu;

end