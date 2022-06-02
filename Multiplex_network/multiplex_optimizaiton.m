clear all
cvx_solver sedumi 
n = 6;%dimension of matrix
tol = 10^(-6);
thetai = -3:0.1:3;
thetaj = -3:0.1:3;
bN = 3;%max number of neighbours
solution = [];
%loop on transformation marix parameters
for i = 1:size(thetai,2)
    thetaii = thetai(i);
    for j = 1:size(thetaj,2) 
        thetajj = thetaj(j);
            T = [eye(2) thetaii*eye(2) zeros(2); zeros(2) eye(2) thetajj*eye(2); zeros(2) zeros(2) eye(2)];
    %solve optimization problem for each value theta
cvx_begin sdp quiet
        variables k0 k1 k2 g0 g1 g2 bsigma d %bsigma:=\bar{\sigma}; d:=\underline{\sigma}/2/bN
        minimize  -g0-g1-g2
        subject to:
            TAT = T*[-k0*eye(2) eye(2) zeros(2); -k1*eye(2) zeros(2) eye(2); -k2*eye(2) zeros(2) zeros(2)]*T^(-1);
            TBiiT = T*[-bN*g0*eye(2) zeros(2) zeros(2); -bN*g1*eye(2) zeros(2) zeros(2); -bN*g2*eye(2) zeros(2) zeros(2)]*T^(-1);
            TBijT = T*[g0*eye(2) zeros(2) zeros(2); g1*eye(2) zeros(2) zeros(2); g2*eye(2) zeros(2) zeros(2)]*T^(-1);
            
            (TAT+TAT')/2+bsigma*eye(n) <= -eye(n)*0;
            [bN*d*eye(n) TBiiT';TBiiT bN*d*eye(n)] >= 0;
            [d*eye(n) TBijT';TBijT d*eye(n)] >= 0;
            
            k0 >= 0;
            k1 >= 0;
            k2 >= 0;
            %k0-2*k1 >= 0; %uncomment for robotarium harware parameters
            %k0-2*k2 >= 0; %uncomment for robotarium harware parameters
            g0 >= 0;
            g1 >= 0; 
            g2 >= 0;
            %g0-2*g1 >= 0; %uncomment for robotarium harware parameters
            %g0-2*g2 >= 0; %uncomment for robotarium harware parameters
            
            k0+g0 >=tol;
            k1+g1 >=tol;
            k2+g2 >=tol;

            bsigma >= tol;
            d >= 0;   
            bsigma-2*bN*d >= tol;
cvx_end 
    
    %save if feasible
if cvx_optval ~= Inf 
        solution = [solution; k0 k1 k2 g0 g1 g2 bsigma 2*bN*d thetaii thetajj];
end
    end
end

% plot control gain results, the optimal set of para is visual in the plot
[a,b] = max(solution(:,4)+solution(:,5)+solution(:,6));
fprintf('k0=%.4f\n',solution(b,1))
fprintf('k1=%.4f\n',solution(b,2))
fprintf('k2=%.4f\n',solution(b,3))
fprintf('g0=%.4f\n',solution(b,4))
fprintf('g1=%.4f\n',solution(b,5))
fprintf('g2=%.4f\n',solution(b,6))
figure
plot(1:length(solution(:,1)),solution(:,4)+solution(:,5)+solution(:,6),'o')
hold on
plot(b,solution(b,4)+solution(b,5)+solution(b,6),'x')
xlabel('iterations')
