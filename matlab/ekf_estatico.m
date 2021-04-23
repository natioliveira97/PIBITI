% Esse filtro de kalman considera que as posiçoes são constantes entre um
% uma mediçao e a outra.

%State Transition Matrix
F = eye(7); 

%Process Noise Matriz
qp = 0.001^2;
qq = 0.0001^2;
Q = diag([qp, qp, qp, qq, qq, qq, qq]);

%Measurement Noise Matriz (calculada a partir de um ensaio estatico)
[m_cov, time_cov, error_cov] = load_measures('ensaio_estatico.bag');
R = cov(transpose(m_cov));

%Observation Matrix
H = eye(7);

%Estimate Uncertainty (Covariance Matrix)
P = eye(7);


[m, time, error] = load_measures('ensaio_dinamico.bag');
[row,col] = size(m);

clearvars estimation prediction
estimation(:,1) = m(:,1);
prediction(:,1) = m(:,1);

for i=2:col
    
    prediction(:,i) = F*estimation(:,i-1);
    P = F*P*F'+Q;
    
    K = P*H'*inv(H*P*H'+R);
    
    v = (m(:,i)-H*prediction(:,i));
    S = H*P*H'+R;
    d(:,i) = v'*inv(S)*v;
    
    estimation(:,i) = prediction(:,i)+K*(m(:,i)-H*prediction(:,i));
    P = (eye(7)-K*H)*P*(eye(7)-K*H)'+K*R*K';
    
end


figure
subplot(3,1,1)
plot(time, m(1,:), 'r')
hold on
plot(time,estimation(1,:), 'g')
title('Posição em X')
subplot(3,1,2)
plot(time, m(2,:), 'r')
hold on
plot(time,estimation(2,:), 'g')
title('Posição em Y')
subplot(3,1,3)
plot(time, m(3,:), 'r')
hold on
plot(time,estimation(3,:), 'g')
title('Posição em Z')

figure
subplot(4,1,1)
plot(time, m(4,:), 'r')
hold on
plot(time,estimation(4,:), 'g')
title('Quaternion X')
subplot(4,1,2)
plot(time, m(5,:), 'r')
hold on
plot(time,estimation(5,:), 'g')
title('Quaternion Y')
subplot(4,1,3)
plot(time, m(6,:), 'r')
hold on
plot(time,estimation(6,:), 'g')
title('Quaternion Z')
subplot(4,1,4)
plot(time, m(7,:), 'r')
hold on
plot(time,estimation(7,:), 'g')
title('Quaternion W')

figure
plot(d)
hold on
plot(xlim,14.07*ones(1,2))



