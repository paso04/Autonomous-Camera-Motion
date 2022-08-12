load('auto_matrix.mat')
load('clutch_matrix.mat')
load('scan_matrix.mat')

sizeC = size(clutch_matrix);
sizeS = size(scan_matrix);
sizeA = size(auto_matrix);

nStitchesClutch = sizeC(1)*sizeC(2)-nnz(isnan(clutch_matrix));
nStitchesScan = sizeS(1)*sizeS(2)-nnz(isnan(scan_matrix));
nStitchesAuto = sizeA(1)*sizeA(2)-nnz(isnan(auto_matrix));

% maxNumber = max([nStitchesAuto,nStitchesClutch,nStitchesScan]);

clutch_matrix(isnan(clutch_matrix)) = 0;
scan_matrix(isnan(scan_matrix)) = 0;
auto_matrix(isnan(auto_matrix)) = 0;

totC = sum(sum(clutch_matrix));
totS = sum(sum(scan_matrix));
totA = sum(sum(auto_matrix));

accuracy_clutch = totC/nStitchesClutch;
accuracy_scan = totS/nStitchesScan;
accuracy_auto = totA/nStitchesAuto;


fprintf('\nAccuracy for camera pedal control: %f\n', accuracy_clutch)
fprintf('Accuracy for SCAN control: %f\n', accuracy_scan)
fprintf('Accuracy for autonomous camera control: %f\n\n', accuracy_auto)