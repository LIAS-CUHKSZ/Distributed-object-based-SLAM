[NEES_pose1, NEES_orientation1] = IEKF_Nees_evaluation( X_iekf_set, P_iekf_set, real_path_select);
NEES_pose=NEES_pose1(10:end);
NEES_orientation=NEES_orientation1(10:end);
step = 1:length(NEES_pose);

subplot(1,2,1);
plot(step,NEES_pose);
xlabel("step");
ylabel("NEES pose")
subplot(1,2,2);
plot(step,NEES_orientation);
xlabel("step");
ylabel("NEES orientation")