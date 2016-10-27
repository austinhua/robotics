figure(1);
plot(knn, PRMRuntimes);
xlabel('knn');
ylabel('PRM Runtime (s)');
title('Varying knn for PRM');

figure(2);
plot(connectionThreshold, PRMRuntimes1);
xlabel('connectionThreshold');
ylabel('PRM Runtime (s)');
title('Varying connectionThreshold for PRM');

figure(3);
plot(connectionThreshold, RRTRuntimes);
xlabel('connectionThreshold');
ylabel('RRT Runtime (s)');
title('Varying connectionThreshold for RRT');

figure(4);
plot(perturbationThreshold, RRTRuntimes1);
xlabel('perturbationThreshold');
ylabel('RRT Runtime (s)');
title('Varying perturbationThreshold for RRT');
