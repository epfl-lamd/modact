classdef CS1 < PROBLEM
% <problem> <MODAct>
% MODACT CS1
%
% Before running this problem, you need to initialize python (once per session)
% pyversion /path/to/python/environment/with/modact
%--------------------------------------------------------------------------
    properties(SetAccess = private)
        problem; % The current python Problem object
        PopCon; % Store the constraints
    end
    methods
        %% Initialization
        function obj = CS1()
            obj.problem = py.modact.problems.get_problem('cs1');
            obj.Global.M = length(obj.problem.weights);
            b = obj.problem.bounds();
            xl = double(b{1});
            xu = double(b{2});
            obj.Global.D = length(xl);
            obj.Global.lower    = xl;
            obj.Global.upper    = xu;
            obj.Global.encoding = 'real';
        end
        %% Calculate objective values
        function PopObj = CalObj(obj,X)
            PopObj = zeros(size(X, 1), obj.Global.M);
            PopCon = zeros(size(X, 1), length(obj.problem.c_weights));
            for i=1:size(X,1)
               out = obj.problem(X(i,:));
               PopObj(i,:) = -1*cell2mat(cell(out{1})).*cellfun(@double,cell(obj.problem.weights));
               PopCon(i,:) = cell2mat(cell(out{2})).*cellfun(@double,cell(obj.problem.c_weights));
            end
            obj.PopCon = PopCon;
        end
        %% Calculate constraint violations
        function PopCon = CalCon(obj,X)  
            PopCon = obj.PopCon;
        end
    end
end