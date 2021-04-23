load('medidas.mat');
%State Transition Matrix
F = eye(7); 

%Process Noise Matriz
qp = 0.01^2;
qq = 0.001^2;
Q = diag([qp, qp, qp, qq, qq, qq, qq])*10;

%Measurement Noise Matriz
R = cov(transpose(measures_static));

%Observation Matrix
H = eye(7);

%Estimate Uncertainty (Covariance Matrix)
P = eye(7);
[row,col]=size(measures);

clearvars estimation prediction
estimation(:,1) = measures(:,1);
prediction(:,1) = measures(:,1);

for i=2:col
    
    prediction(:,i) = F*estimation(:,i-1);
    P = F*P*transpose(F)+Q;
    
    K = P*transpose(H)*inv(H*P*transpose(H)+R);
    v = (measures(:,i)-H*prediction(:,i));
    S = H*P*transpose(H)+R;
    d(:,i) = v'*inv(S)*v;
    estimation(:,i) = prediction(:,1)+K*(measures(:,i)-H*prediction(:,i));
    P = (eye(7)-K*H)*P*transpose(eye(7)-K*H)+K*R*transpose(K);
    
end

plot(measures(1,:))
hold on
plot(estimation(1,:))

%figure
%plot(d)
%hold on
%plot(xlim,14.07*ones(1,2))



