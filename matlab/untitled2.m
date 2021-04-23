%Esse filtro considera que a velocidade entre uma medida e a outra de
%posição é constante, já os quaternions o modelo considera que não há
%velocidade de rotação

%Process Noise Matriz
Q = eye(2)*0.1;

%Measurement Noise Matriz (calculada a partir de um ensaio com a tag
%parada)
[m_cov, time_cov, error_cov] = load_measures('ensaio_estatico.bag');
R = cov(transpose(m_cov));
R = [0.001];

%Observation Matrix
H = [1 0;
     0 0] ;

%Estimate Uncertainty (Covariance Matrix)
P = eye(2);

%Carrega medidas do ensaio dinamico
[m, time, error]=load_measures('ensaio_dinamico.bag');
[row,col]=size(m);

m = m(1,:);
clearvars estimation prediction
estimation(:,1) = [m(1,1),0];
prediction(:,1) = [m(1,1),0];

for i=2:col
    dt = time(i)-time(i-1);
    
    
    %State Transition Matrix
    %É calculada em cada interação porque o intervalo entre medidas não é fixo.
    F = [1 dt;
         0 1 ];
    
    % Equações do filtro de kalman
    prediction(:,i) = F*estimation(:,i-1);
    P = F*P*inv(F)+Q;
    
    K = P*transpose(H)*inv(H*P*transpose(H)+R);
    
    estimation(:,i) = prediction(:,i)+K*(m(:,i)-H*prediction(:,i));
    P = (eye(2)-K*H)*P*transpose(eye(2)-K*H)+K*R*transpose(K);
    
    %estimation(:,i);
    %K;
    %P;
    %pause;
end

figure
plot(time, m(1,:), 'r')
hold on
plot(time,estimation(1,:), 'g')
title('Posição em X')




figure
plot(time, estimation(2,:), 'r')
title('Velocidade X')

