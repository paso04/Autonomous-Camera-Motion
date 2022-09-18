load('autoNASATLX.mat')
load('clutchNASATLX.mat')
load('scanNASATLX.mat')

metrics = {'Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration', 'Weighted Rating'};

for i = 1:length(metrics)
    % P = camera pedal, C = SCAN, A = gesture-based
    veca=auto(:,i);
    vecc=clutch(:,i);
    vecs=scan(:,i);
    p_value_ac = signrank(veca{:,:},vecc{:,:});
    p_value_sc = signrank(vecs{:,:},vecc{:,:});
    p_value_as = signrank(veca{:,:},vecs{:,:});
    figure()
    boxplot([vecc{:,:},vecs{:,:},veca{:,:}], 'Labels', {'Camera Pedal', 'SCAN', 'Gesture-Based'});
    title(metrics{i});
    ylabel('Adjusted value')
    title(metrics{i})

    if p_value_sc < 0.001
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([1 2]), [1 1]*max(yt)*1.05, '-k',  mean(xt([1 2])), max(yt)*1.06, '*k',  mean(xt([1 2]))*1.05, max(yt)*1.06, '*k',  mean(xt([1 2]))*0.95    , max(yt)*1.06, '*k')
        hold off
    elseif p_value_sc < 0.01
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([1 2]), [1 1]*max(yt)*1.05, '-k',  mean(xt([1 2])), max(yt)*1.06, '*k',  mean(xt([1 2]))*1.05, max(yt)*1.06, '*k')
        hold off
    elseif p_value_sc < 0.05
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([1 2]), [1 1]*max(yt)*1.05, '-k',  mean(xt([1 2])), max(yt)*1.06, '*k')
        hold off
    end
    if p_value_as < 0.001
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([2 3]), [1 1]*max(yt)*1.05, '-k',  mean(xt([2 3])), max(yt)*1.06, '*k',  mean(xt([2 3]))*1.05, max(yt)*1.06, '*k',  mean(xt([2 3]))*0.95    , max(yt)*1.06, '*k')
        hold off
    elseif p_value_as < 0.01
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([2 3]), [1 1]*max(yt)*1.05, '-k',  mean(xt([2 3])), max(yt)*1.06, '*k',  mean(xt([2 3]))*1.05, max(yt)*1.06, '*k')
        hold off
    elseif p_value_as < 0.05
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([2 3]), [1 1]*max(yt)*1.05, '-k',  mean(xt([2 3])), max(yt)*1.06, '*k')
        hold off
    end
    if p_value_ac < 0.001
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.1)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([1 3]), [1 1]*max(yt)*1.075, '-k',  mean(xt([1 3])), max(yt)*1.09, '*k',  mean(xt([1 3]))*1.05, max(yt)*1.09, '*k',  mean(xt([1 3]))*0.95    , max(yt)*1.09, '*k')
        hold off
    elseif p_value_ac < 0.01
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.2)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([1 3]), [1 1]*max(yt)*1.075, '-k',  mean(xt([1 3])), max(yt)*1.09, '*k',  mean(xt([1 3]))*1.05, max(yt)*1.09, '*k')
        hold off
    elseif p_value_ac < 0.05
        yt = get(gca, 'YTick');
        axis([xlim    0  ceil(max(yt)*1.2)])
        xt = get(gca, 'XTick');
        hold on
        plot(xt([1 3]), [1 1]*max(yt)*1.075, '-k',  mean(xt([1 3])), max(yt)*1.09, '*k')
        hold off
    end
end
