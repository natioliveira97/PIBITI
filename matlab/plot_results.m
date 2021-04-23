 [m1, time1] = results('testrohorantihor.bag', '/fiducial_transforms');
 [m2, time2] = results('testrohorantihor.bag', '/fiducial_transforms_filtered');
 

 [a, size1] = size(time1);
 [a,size2] = size(time2);
 
 n=size1

for i=1:size1
    
    if time1(1,i)>time2(size2)
        n=i;
       break
    else
        
    end
end

time = time1(1:n); 
m = m1(1:7,1:n);

 
figure
subplot(3,1,1)
plot(time, m(1,:), 'r')
hold on
plot(time2, m2(1,:), 'g')
title('Posição em X')
subplot(3,1,2)
plot(time, m(2,:), 'r')
hold on
plot(time2, m2(2,:), 'g')
title('Posição em Y')
subplot(3,1,3)
plot(time, m(3,:), 'r')
hold on
plot(time2, m2(3,:), 'g')
title('Posição em Z')
legend('Algoritmo de detecção', 'Saída do filtro')

figure
subplot(4,1,1)
plot(time, m(4,:), 'r')
hold on
plot(time2, m2(4,:), 'g')
title('Quaternion X')
subplot(4,1,2)
plot(time, m(5,:), 'r')
hold on
plot(time2, m2(5,:), 'g')
title('Quaternion Y')
subplot(4,1,3)
plot(time, m(6,:), 'r')
hold on
plot(time2, m2(6,:), 'g')
title('Quaternion Z')
subplot(4,1,4)
plot(time, m(7,:), 'r')
hold on
plot(time2, m2(7,:), 'g')
title('Quaternion W')

legend('Algoritmo de detecção', 'Saída do filtro')