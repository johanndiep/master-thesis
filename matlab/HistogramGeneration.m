% Johann Diep (jdiep@student.ethz.ch) - April 2019

% This program generates histograms for range visualization purpose per
% anchor.

%% Histogram function

function HistogramGeneration = HistogramGeneration(range_array)
    for j = 1:8
        [range_outliers_removed, TF] = rmoutliers(range_array(j,:)); % outlier removal

        subplot(2, 4, j);
        histogram(range_outliers_removed, 50, 'FaceColor', 'y');

        title("Anchor " + j + ": " + mean(range_outliers_removed)/1000 + " m", 'FontWeight', 'normal');
    end
end