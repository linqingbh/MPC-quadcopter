function [Phi_Phi,Phi_F,Phi_R,A_e,B_e,C_e,Phi_D,F,Phi] = mpcgain_mimo(A,B,H,Nc,Np,D)

global data

[m1,n1]=size(H);
[nb,n_in]=size(B);
[nd,nd_in]=size(D);
[y1 y2]=size(H*B);
[q,q1]=size(A);
A_e=zeros(q+m1,q+m1);
A_e(1:q,1:q)=A;
A_e(q+1:q+m1,1:n1)=H*A;
A_e(q+1:m1+q,q+1:q+m1)=eye(m1);

% B_e(1:nb,1:n_in)=B
B_e=B;
B_e(nb+1:nb+y1,1:y2)=H*B;

[y1 y2]=size(H*D);
D_e=D;
D_e(nd+1:nd+y1,1:y2)=H*D;

C_e=zeros(m1,n1);
C_e(1:m1,n1+1:n1+m1)=eye(m1);


[x1,x2]=size(C_e*A_e);
for kk=1:Np
    nn=kk-1;
F(x1*nn+1:x1*nn+x1,1:x2)=C_e*A_e^kk;
end
[x3,x4]=size(C_e*B_e);
for i=1:Nc
    mm=i-1;
    for j=1:Np
        nn=j-1;
        if j<i
         Phi(x3*nn+1:x3*nn+x3,x4*mm+1:x4*mm+x4)=zeros(x3,x4);
        else
          Phi(x3*nn+1:x3*nn+x3,x4*mm+1:x4*mm+x4)=C_e*A_e^(j-i)*B_e;
        end
    end
end

[x3,x4]=size(C_e*D_e);
for i=1:Nc
    mm=i-1;
    for j=1:Np
        nn=j-1;
        if j<i
         Phi_d(x3*nn+1:x3*nn+x3,x4*mm+1:x4*mm+x4)=zeros(x3,x4);
        else
          Phi_d(x3*nn+1:x3*nn+x3,x4*mm+1:x4*mm+x4)=C_e*A_e^(j-i)*D_e;
        end
    end
end

[n,m]=size(C_e);
% W=[eye(m) zeros(m,Nc)];
% BarRs=eye(n,m+n);%[0 1 0 1 0 1 0 1]';%ones(Np*2,1);%1
[x1,x2]=size(Phi);
Phi_Phi= Phi'*Phi;
Phi_F= Phi'*F;
Phi_D= Phi'*Phi_d;
[x1,x2]=size(Phi_F);
Phi_R=Phi_F(1:x1,x2-m1+1:x2);
[x1,x2]=size(Phi_D);
Phi_D=10*Phi_D(:,1);
% Phi_D=Phi_D(1:x1,x2-m1+1:x2);
end