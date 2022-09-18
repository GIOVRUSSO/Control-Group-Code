clear all
cvx_solver sedumi 
n = 3;%dimension of matrix
tol = 10^(-6);
alpha = -1:0.1:1;
beta = -1:0.1:1;
solution = [];
%solve the optimisation problem for a grid of parameters alpha and beta
for i = 1:size(alpha,2)
    alphai = alpha(i);
    for j = 1:size(beta,2) 
        betaj = beta(j);
            T = [1 alphai 0; 0 1 betaj; 0 0 1];
    %solve optimization problem for each value theta
cvx_begin sdp quiet
        variables k0 k1 k2 g0 g1 g2 bsigma d c
        minimize  -g0-g1-g2
        subject to
            Aii = T*[-0.1-1*k0 1 0; -k1 0 1; -k2 0 0]*T^(-1);
            Aij = T*[0.05 0 0; 0 0 0; 0 0 0]*T^(-1);
            Bii = T*[-1*g0 0 0; -g1 0 0; -g2 0 0]*T^(-1);
            Bij = T*[1*g0 0 0; g1 0 0; g2 0 0]*T^(-1);
            
            (Aii+Aii')/2+bsigma*eye(n) <= -eye(n)*0;
            [d*eye(n) Bij';Bij d*eye(n)] >= 0;
            [d*eye(n) Bii';Bii d*eye(n)] >= 0;
            [c*eye(n) Aij';Aij c*eye(n)] >= 0;
            
            k0 >= 0;
            k1 >= 0;
            k2 >= 0;
            g0 >= 0;
            g1 >= 0;
            g2 >= 0;

            bsigma >= tol;
            d >= 0;   
            c >= 0;
            bsigma-4*d-2*c >= tol;%the matrix measure in condition C2 should be negative enough to compensate the norms of A_{ij} abd (B_k)_{ij}
cvx_end 
    
    %save if feasible
if cvx_optval ~= Inf 
        solution = [solution; k0 k1 k2 g0 g1 g2 bsigma 4*d alphai betaj];
end
    end
end