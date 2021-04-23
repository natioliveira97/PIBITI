%State Transition Matrix
F = eye(7); 

%Process Noise Matriz
Q = eye(7)*0.0001;

%Measurement Noise Matriz
[m_cov, time_cov, error_cov] = load_measures('ensaio_estatico.bag');
R = cov(transpose(m_cov));

%Observation Matrix
H = eye(7);

%Estimate Uncertainty (Covariance Matrix)
P = eye(7)*0.001;

[m,time,error]=load_measures('v_cons.bag');


estimation(:,1) = m(:,1);
prediction(:,1) = m(:,1);


[row,col] =size(m);

for i=2:col
    
    prediction(:,i) = F*estimation(:,i-1);
    P = F*P*inv(F)+Q;
    
    K = P*transpose(H)*inv(H*P*transpose(H)+R);
    
    estimation(:,i) = prediction(:,1)+K*(m(:,i)-H*prediction(:,i));
    P = (eye(7)-K*H)*P*transpose(eye(7)-K*H)+K*R*transpose(K);
    
end



