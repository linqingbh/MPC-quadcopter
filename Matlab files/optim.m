function x=optim(E,F,M,gamma,DeltaU);
global data
[n1,m1]=size(M);
x=DeltaU;
kk=0;
for i=1:n1
    if (M(i,:)*x>gamma(i)) 
        kk=kk+1;
    else
        kk=kk+0;
    end
end
if (kk==0) 
    return; 
end
H=M*inv(E)*M';
K=(M*inv(E)*F+gamma);
[n,m]=size(K);
x_ini=zeros(n,m);
lambda=x_ini;
al=10;
for km=1:38
    %find the elements in the solution vector one by one
    % km could be larger if the Lagranger multiplier has a slow
    % convergence rate.
    lambda_p=lambda;
    for i=1:n
        w= H(i,:)*lambda-H(i,i)*lambda(i,1);
        w=w+K(i,1);
        la=-w/H(i,i);
        lambda(i,1)=max(0,la);
    end
        al=(lambda-lambda_p)'*(lambda-lambda_p);
    if (al<10e-8); 
        break; 
    end
end
x=DeltaU-inv(E)*M'*lambda;