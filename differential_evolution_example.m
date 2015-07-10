%%
% A sample implementation of the differential evolution algorithm.
%
% For costly to evaluate cost functions, such as in the F1 optimization
% example, one should save the cost-function evaluation results for a
% factor of 2 speed improvement.

% Set population size and the dimensionality of the cost function.
N = 100;
n = 2;

% Define cross-over rate.
CR = 0.1;
% Define gain for mutations.
F = 1;

% The sample cost function, with a ill-behaving boundary.
f = @(x) x*x' + (x*x' > 0.05 & x*x' < 0.5) * 1000;

% Initial population.
X = randn(N,n);

% Initialize visualization.
figure(1); clf;
plot(X(:,1),X(:,2),'+');
axis([-1 1 -1 1]);

for loop = 1:1000 ,
    for i = 1:N ,
        % Pick distictive a, b, and c.
        set = setdiff(1:N,i);
        set = set(randperm(length(set)));
        set = set(1:3);
        % Perform reproduction.
        R = randi([1 n]);
        r = rand(1,n) < CR;
        r(R) = 1;
        y = X(i,:);
        y(r) = X(set(1),r) + F * (X(set(2),r) - X(set(3),r));
        % If improvement, replace ancestor.
        if f(y) < f(X(i,:)) ,
            X(i,:) = y;
        end
    end
    
    % Some visualization.
    plot(X(:,1),X(:,2),'+');
    axis([-1 1 -1 1]);
    title(sprintf('%d', loop));
    pause(.02);
end
