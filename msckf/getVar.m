% Try to figure out appropriate variances from known linear-ish motion

% load('../datasets/2015-02-04-12-54-12_chairwall2.mat')
format long

startInd = 10;
endInd = 116;

v = v_vk_vk_i(:,startInd:endInd);
w = w_vk_vk_i(:,startInd:endInd);
k = 1:size(v,2);

vfit = zeros(size(v));
wfit = zeros(size(w));

pv1 = polyfit(k,v(1,:),1)
pv2 = polyfit(k,v(2,:),1)
pv3 = polyfit(k,v(3,:),1)

pw1 = polyfit(k,w(1,:),1)
pw2 = polyfit(k,w(2,:),1)
pw3 = polyfit(k,w(3,:),1)

vfit(1,:) = polyval(pv1,k);
vfit(2,:) = polyval(pv2,k);
vfit(3,:) = polyval(pv3,k);

wfit(1,:) = polyval(pw1,k);
wfit(2,:) = polyval(pw2,k);
wfit(3,:) = polyval(pw3,k);

figure(1); clf;
subplot(3,1,1); hold on;
plot(k,v(1,:),'-b'); plot(k,vfit(1,:),'-r');
subplot(3,1,2); hold on;
plot(k,v(2,:),'-b'); plot(k,vfit(2,:),'-r');
subplot(3,1,3); hold on;
plot(k,v(2,:),'-b'); plot(k,vfit(2,:),'-r');

figure(2); clf;
subplot(3,1,1); hold on;
plot(k,w(1,:),'-b'); plot(k,wfit(1,:),'-r');
subplot(3,1,2); hold on;
plot(k,w(2,:),'-b'); plot(k,wfit(2,:),'-r');
subplot(3,1,3); hold on;
plot(k,w(2,:),'-b'); plot(k,wfit(2,:),'-r');

verr = v-vfit;
werr = w-wfit;

vvar = var(verr')
wvar = var(werr')
