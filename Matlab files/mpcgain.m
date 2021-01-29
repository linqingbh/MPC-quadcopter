function [Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e,F,Phi] = mpcgain(A,B,H,Nc,Np)

global data

[m1,n1]=size(H);
[n1,n_in]=size(B);
A_e=eye(n1+m1,n1+m1);
A_e(1:n1,1:n1)=A;
A_e(n1+1:n1+m1,1:n1)=H*A;
B_e=zeros(n1+m1,n_in);
B_e(1:n1,:)=B;
B_e(n1+1:n1+m1,:)=H*B;
C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);
n=n1+m1;
h(1,:)=C_e;
F(1,:)=C_e*A_e;
for kk=2:Np
    h(kk,:)=h(kk-1,:)*A_e;
    F(kk,:)= F(kk-1,:)*A_e;
end
    v=h*B_e;
    Phi=zeros(Np,Nc); %declare the dimension of Phi
    Phi(:,1)=v; % first column of Phi
for i=2:Nc
    Phi(:,i)=[zeros(i-1,1);v(1:Np-i+1,1)]; %Toeplitz matrix
end
    BarRs=ones(Np,1);
    Phi_Phi= Phi'*Phi;
    Phi_F= Phi'*F;
    Phi_R=Phi'*BarRs;
end