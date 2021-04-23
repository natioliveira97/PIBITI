function plot_measures(bag1, bag2)
    [m, time, erro] = load_measures_from_bag(bag1);
    [m2, time2, erro2] = load_measures_from_bag(bag2);

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

end