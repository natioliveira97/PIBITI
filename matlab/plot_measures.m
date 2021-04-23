function plot_measures(bag)
    [m, time, erro] = load_measures_from_bag(bag);

figure
subplot(3,1,1)
plot(time, m(1,:), 'r')
title('Posição em X')
subplot(3,1,2)
plot(time, m(2,:), 'r')
title('Posição em Y')
subplot(3,1,3)
plot(time, m(3,:), 'r')
title('Posição em Z')

figure
subplot(4,1,1)
plot(time, m(4,:), 'r')
title('Quaternion X')
subplot(4,1,2)
plot(time, m(5,:), 'r')
title('Quaternion Y')
subplot(4,1,3)
plot(time, m(6,:), 'r')
title('Quaternion Z')
subplot(4,1,4)
plot(time, m(7,:), 'r')
title('Quaternion W')

end