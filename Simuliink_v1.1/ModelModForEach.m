classdef (StrictDefaults) ModelModForEach < matlab.System & matlab.system.mixin.Propagates
%Modified to work with for each loops

%Model System block that allows reconfiguration into
% different Motion Models supported by
% robotics.core.internal.navigation.ModelFactory

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    properties (Nontunable)
        %ModelName - Name for the motion model instance
        %   0 is a place holder default value.
        %   Use numeric value to let System Block allow using MATLAB
        %   variable for this parameter
        ModelName = 0;

        %ModelType - Defines the motion model type.
        %   Motion Model Type must be supported by the factory
        ModelType = 0;

        %ModelConfiguration - Defines all the configuration parameters used by the motion model
        %   0 is a place holder for the configuration struct
        %   The configuration struct would contain different fields when
        %   model type is changed
        ModelConfiguration = 0;

        %DataType - Defines the output data type of the step method
        DataType = 0;

        %UseExternalConfiguration - Flag defines whether the configuration
        %is accepted as additional input to step function
        UseExternalConfiguration = false;
    end

    properties (Access = private)
        %ModelImpl - model implementation created according to ModelType and DataType
        ModelImpl
    end

    methods
        function out = copy(in)
        %copy implements deep copy syntax
            out = robotics.core.internal.system.navigation.Model;
            for fieldName = fieldnames(in)'
                out.(fieldName{1}) = in.(fieldName{1});
            end
            out.ModelImpl = in.ModelImpl;
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function m = getModelImpl(obj)
            m = obj.ModelImpl;
        end

        function setModelImpl(obj, modelMock)
            obj.ModelImpl = modelMock;
        end
    end

    methods (Access = protected)

        function setupImpl(obj, ~, control, environment, config)
        %setupImpl setup method for the block
        %   Setup is executed during model initialization. The system
        %   object setup the model implementation.

            obj.reset();

            % validate the input structs
            basemodel = robotics.core.internal.navigation.ModelFactory.getMotionModel(obj.ModelType, obj.DataType);
            obj.validateStruct(control, basemodel.control('struct', obj.DataType));
            obj.validateStruct(environment, basemodel.environment('struct', obj.DataType));

            if nargin == 5
                obj.validateStruct(config, basemodel.Configuration.toStruct(obj.DataType));
            end
        end

        function resetImpl(obj)
        %resetImpl initialize / reset discrete-state properties

            obj.ModelImpl = robotics.core.internal.navigation.ModelFactory.getMotionModel(obj.ModelType, obj.DataType);
            obj.ModelImpl.Name = string(obj.ModelName);
            obj.ModelImpl.Configuration = obj.ModelImpl.Configuration.fromStruct(obj.ModelConfiguration, obj.DataType);
        end

        function stateDerivative = stepImpl(obj, state, control, environment, config)
        %stepImpl implements the Motion Model step method
        %   Step computes the time derivative of the states defined by
        %   the motion model

            if nargin == 5
                obj.ModelImpl.Configuration = obj.ModelImpl.Configuration.fromStruct(config, obj.DataType);
            end

            stateDerivative = cast(derivative(obj.ModelImpl, state, control, environment), obj.DataType);
        end

        function num = getNumInputsImpl(obj)
        %getNumInputsImpl Define total number of inputs for system with optional inputs

        % Allow for additional input port when external configuration is
        % used
            if obj.UseExternalConfiguration
                num = 4;
            else
                num = 3;
            end
        end

        function out = isOutputComplexImpl(~)
        %isOutputComplexImpl output must be real
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
        %isOutputFixedSizeImpl output must be fixed sized
            out = true;
        end

        function flag = supportsMultipleInstanceImpl(obj)
            % Return true if System block can be used inside a For Each
            % subsystem, which requires multiple object instances
            flag = true;
        end


        function out = getOutputSizeImpl(obj)
        %getOutputSizeImpl output size must be coherent with the model state size

        % The propagation is currently ahead of the block setup call.
        % Thus the obj.Model is still not initialized. This function
        % has to create a local model.

            model = robotics.core.internal.navigation.ModelFactory.getMotionModel(obj.ModelType, obj.DataType);
            out = size(state(model, 'vector', obj.DataType));
        end

        function out = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl output data type is defined by non-tunable obj.DataType
            out = obj.DataType;
        end
    end

    methods (Static, Access=private)
        function validateStruct(input, baseline)

            fieldNamesBaseline = fieldnames(baseline);
            fieldNames = fieldnames(input);

            if (numel(fieldNames) ~= numel(fieldNamesBaseline))
                coder.internal.errorIf(true, 'shared_robotics:robotcore:navigationmodel:MismatchInputStruct');
            end

            for i = 1:numel(fieldNames)
                if ~strcmp(fieldNames{i}, {fieldNamesBaseline{i}})
                    coder.internal.errorIf(true, 'shared_robotics:robotcore:navigationmodel:MismatchInputStruct');
                end

                basefield = baseline.(fieldNamesBaseline{i});
                inputfield = input.(fieldNames{i});
                if ~isa(inputfield, class(basefield)) ...
                        || ~isequal(size(inputfield), size(basefield))
                    coder.internal.errorIf(true, 'shared_robotics:robotcore:navigationmodel:MismatchInputStruct');
                end
            end

        end
    end

end
