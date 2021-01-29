function simulator_init
    global data    
    %% Initialize TrueTime kernel    
    ttInitKernel('prioEDF'); % nbrOfInputs, nbrOfOutputs, EDF
    
    data.t=0;
    data.t_vector=[];
    data.deadline = 0.5;
    data.computing_time=0e-6;    

    %% Initzialization
    
    % tunning parameters
    data.Nc=8;  % control horizon
    data.Np=46; % prediction horizon
    R = 0.01;   % control weighting
    
    %% State-space model
    data.Am=[1.9512 -0.9512 0.0036;1 0 0;0 0 0]; 
    data.Bm=[0.0037;0;1]; 
    data.Cm=[1 0 0];
    
    %% Init control, reference and output signal   
    data.u=0;
    data.u_vector=[];
    data.r_vector=[];    
    data.y_vector=[];
    % Init system states
    data.xm=[0 0 0]'; 
    data.Xf=[0 0 0 0]'; % augmented incremetal state [deltax y]'
  
    %% Get the augmented incremental model and the parameters of the incremental trajectory control
    [data.GG, data.GF, data.GR, data.A, data.B, data.C , data.F, data.G]=mpcgain(data.Am,data.Bm,data.Cm,data.Nc,data.Np);
    
    %% Constant part of the incremental trajectory control
    data.f1=data.GG+R*eye(data.Nc,data.Nc);
    
    %% Constraints parameters
    data.ymax=(pi/6)*1.02;  % output max limit
    data.ymin=(pi/6)*0.98;  % output min limit
    data.M=[data.G(1,:);-data.G(1,:)]; % Constrain matrix M
    
    ttCreateTask('controller_task',data.deadline, 1, 'controller', data);%,deadline,prio,'code',data);
    ttCreateJob('controller_task');
end
