%Esse filtro considera que a velocidade entre uma medida e a outra de
%posição é constante, já os quaternions o modelo considera que não há
%velocidade de rotação

%Process Noise Matriz
qv = 0.01^2;
qq = 0.001^2;
Qa = diag([0, 0, 0, qv, qv, qv, qq, qq, qq, qq]);

%Measurement Noise Matriz (calculada a partir de um ensaio com a tag
%parada)
[m_cov, time_cov, error_cov] = load_measures('ensaio_estatico.bag');
R = cov(transpose(m_cov));
R = [0.0001    0.0000   -0.00    0.0000   -0.0000   -0.0000   -0.0000;
    0.000     0.0007   -0.00    0.0000    0.0000   -0.000   -0.0000;
   -0.000     -0.00    0.0089   -0.0000    0.0000    0.000    0.0000;
    0.0000    0.0000   -0.0000    0.0002    0.0000   -0.0000   -0.0000;
   -0.0000    0.0000    0.0000    0.0000    0.0071   -0.0000   -0.000;
   -0.0000   -0.0000    0.0000   -0.0000   -0.0000    0.2378    0.000;
   -0.0000   -0.0000    0.0000   -0.0000   -0.000    0.000    0.1265];
   
   
R = R*0.0001;
   
   
rp = 0.0001^2;


re = 0.00000001^2;

%Observation Matrix
H = [1 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 1]; 

%Estimate Uncertainty (Covariance Matrix)
P = eye(10);

%Carrega medidas do ensaio dinamico
[m, time, error]=load_measures_from_bag(bSel);
[row,col]=size(m);

clearvars estimation prediction
estimation(:,1) = [m(1,1), m(2,1), m(3,1), 0.1 ,0.1 ,0.1 ,m(4,1), m(5,1), m(6,1), m(7,1)];
prediction(:,1) = [m(1,1), m(2,1), m(3,1), 0.1 ,0.1 ,0.1 ,m(4,1), m(5,1), m(6,1), m(7,1)];

for i=2:col
    dt = time(i)-time(i-1);
    
    last_q = estimation(7:10);
    q1=[m(4,i); m(5,i); m(6,i); m(7,i)];
    q2=-q1;
    
    if norm(last_q-q2) < norm(last_q-q1)
        m(4,i)=-m(4,i);
        m(5,i)=-m(5,i);
        m(6,i)=-m(6,i);
        m(7,i)=-m(7,i);
    end
    
    
    
    
    
    %State Transition Matrix
    %É calculada em cada interação porque o intervalo entre medidas não é fixo.
    F = [1 0 0 dt 0 0 0 0 0 0;
     0 1 0 0 dt 0 0 0 0 0;
     0 0 1 0 0 dt 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 1]; 
    
    % O erro de processo também será calculado a cada interação,
    % considerando o erro estimado para velocidade e para os quaternions
    Q = F*Qa*F';
    
    % Equações do filtro de kalman
    prediction(:,i) = F*estimation(:,i-1);
    P = F*P*inv(F)+Q;
    
    K = P*transpose(H)*inv(H*P*transpose(H)+R);
    
    estimation(:,i) = prediction(:,i)+K*(m(:,i)-H*prediction(:,i));
    P = (eye(10)-K*H)*P*transpose(eye(10)-K*H)+K*R*transpose(K);
    
    % Pseudo medição
    H2 = [0 0 0 0 0 0 2*estimation(7,i) 2*estimation(8,i) 2*estimation(9,i) 2*estimation(10,i)];
    K = P*transpose(H2)*inv(H2*P*transpose(H2)+re);
    estimation(:,i) = estimation(:,i)+K*(1-estimation(7,i)^2-estimation(8,i)^2-estimation(9,i)^2-estimation(10,i)^2);
    P = (eye(10)-K*H2)*P*transpose(eye(10)-K*H2)+K*re*transpose(K);
    
    % D = 1-estimation(7,i)^2-estimation(8,i)^2-estimation(9,i)^2-estimation(10,i)^2
    
    
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
plot(time,estimation(7,:), 'g')
title('Quaternion X')
subplot(4,1,2)
plot(time, m(5,:), 'r')
hold on
plot(time,estimation(8,:), 'g')
title('Quaternion Y')
subplot(4,1,3)
plot(time, m(6,:), 'r')
hold on
plot(time,estimation(9,:), 'g')
title('Quaternion Z')
subplot(4,1,4)
plot(time, m(7,:), 'r')
hold on
plot(time,estimation(10,:), 'g')
title('Quaternion W')

figure
subplot(3,1,1)
plot(time, estimation(4,:), 'r')
title('Velocidade X')
subplot(3,1,2)
plot(time, estimation(5,:), 'r')
title('Velocidade Y')
subplot(3,1,3)
plot(time, estimation(6,:), 'r')
title('Velocidade Z')



