F = eye(7);
H = eye(7);

%Process Noise Matriz
qp = 0.01^2;
qq = 0.001^2;
Q = diag([qp, qp, qp, qq, qq, qq, qq])*10;

R = eye(7)*0.01^2;

P = eye(7);

m = measures;

X_esti(:,1)=m(:,1);
X_pred(:,1)=m(:,1);

for i=2:382
    X_pred(:,i) = F*X_esti(:,i-1);
    P = F*P*F'+Q;
    
    K = P*H'*inv(H*P*H'+R);
    
    X_esti(:,i) = X_pred(:,i)+K*(m(:,i)-H*X_pred(:,i));
    P = (eye(7)-K*H)*P*(eye(7)-K*H)'+K*R*K';
   
end

plot(m(1,:),'r');
hold on;
plot(X_esti(1,:), 'g');

