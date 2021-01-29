
function [exectime, data] = controller(seg, data)
    global data

    switch seg,

    case 1,
    data.t = ttCurrentTime;
    exectime = 0;

    case 2,
    data.t = ttCurrentTime;

    %Read Inputs:
    input=ttAnalogIn(1);

    % reference
    r=pi/6;  
    if(data.t)>20 & (data.t)<23
        r=(pi/6)*1.5;
    end
    if(data.t)>40 & (data.t)<43
        r=(pi/6)*0.5;
    end
    
    % Calculate U
    f2=data.GR*r-data.GF*data.Xf;  % Get the second part of U
    DeltaU=inv(data.f1)*f2; %% Get DeltaU without constrains
    
    % Constrains
    gamma=[data.ymax-data.F(1,:)*data.Xf;-data.ymin+data.F(1,:)*data.Xf];
    %DeltaU=optim(data.f1,-f2,data.M,gamma,DeltaU); 
    
    % Calculate u
    deltau=DeltaU(1,1);  % Apply the receding control horizon (take only the first element)
    data.u=data.u+deltau;
    
    % Get the incremental states vector prediction     
    data.xm_old=data.xm;
    data.xm=data.Am*data.xm+data.Bm*data.u;  
    y=data.Cm*data.xm;     
    data.Xf=[data.xm-data.xm_old;y];
     
    
    % add them to vectors
    data.t_vector=[data.t_vector data.t];
    data.r_vector=[data.r_vector r];
    data.y_vector=[data.y_vector input];
    data.u_vector=[data.u_vector data.u];
    
    exectime = data.computing_time;
  
 case 3,   
    %Output:
    ttAnalogOut(1,data.u);
    
    %Next activation time:
    ttSleepUntil(ttCurrentTime+data.h-data.computing_time);
    exectime = 0;
    
 case 4,
    ttSetNextSegment(2); % loop to segment 2
    exectime = 0;
    
end




