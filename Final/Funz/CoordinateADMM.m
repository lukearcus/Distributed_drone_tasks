classdef CoordinateADMM < matlab.System & matlab.system.mixin.Propagates
    % CoordinateADMM Updates the ADMM vector during the predict cycle
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    properties(DiscreteState)
        
    end
    
    properties(Nontunable)
        N (1,1) double = 30;                                   % time horizon (samples)
        delta (1,1) double = 0.5;                                % distance constraint
        M (1,1) double = 4; % Number of agents
        
        SampleTime = 0.1; % Sample Time
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Inherited"
        OffsetTime = 0.05; % Offset Time
        TickTime = 0.1;
    end
    
    % Pre-computed constants
    properties(Access = private)

        posM
        d
        posMN

    end
    
    properties(Access = private, Nontunable)
        Nd = 3; % number of dimensions
        nx
        nu

    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

            
            obj.nx = 6; %state vector dimension
            obj.nu = 3; %number of inputs
            
            
     
            obj.posM = blkdiag(eye(obj.Nd),zeros(obj.nx-obj.Nd)); % Matrix to take only position from the state vector x (zeros rest)
            obj.d = [eye(obj.Nd),zeros(obj.Nd,obj.nx-obj.Nd)]; % Matrix to take only position from the state vector x (only returns pos)
            obj.posMN = kron(eye(obj.N+1),obj.d); %returns pos across time steps

            
        end
        
        function ADMM_update = stepImpl(obj, ADMM_input, constraints,i)
            ADMM_update = ADMM_input;
            H = kron(eye(obj.N+1),repmat(obj.d, nnz(constraints.N_j), 1));
            Hw = kron(eye(obj.N+1),repmat(eye(obj.nu), nnz(constraints.N_j), 1));
            for j = 1:nnz(constraints.N_j)
                v = zeros(nnz(constraints.N_j),1);
                v(j) = 1;
                H_M = kron(eye(obj.N+1),kron(v, -1*obj.d));
                H_Mw = kron(eye(obj.N+1),kron(v, -1*eye(obj.nu)));
                
                H =  [H, H_M];
                Hw = [Hw,H_Mw];
            end
          
            rhoM = kron(speye((obj.N+1)),ADMM_update.rho/2*eye(obj.nu)); % matrix for the quadratic objective formualation
            Pc = kron(eye(1+nnz(constraints.N_j)),rhoM);
            

            qc = coordination_linear(ADMM_update.lambda,ADMM_update.lambda_to_j,ADMM_update.rho,ADMM_update.x_bar,obj.posMN,constraints.N_j,i);
            
            % Update matrices
            actual_N_j = nonzeros(constraints.N_j);

            init = ADMM_update.w;
            for j = 1:nnz(constraints.N_j)
                init = [init;ADMM_update.w_to_j(:,actual_N_j(j))];
            end
            [A_ineq,l_ineq] = communicate(ADMM_update.x_bar,obj.N,constraints.N_j,H,Hw,constraints.delta,obj.nu,i);
            
            if A_ineq*init < l_ineq
                init = A_ineq\l_ineq;
            end
            opts = optimoptions('quadprog','Display','off','Algorithm','Active-set');
            resc = quadprog(full(Pc),qc,-A_ineq,-l_ineq,[],[],[],[],init,opts);
%             resc = qp_grad(full(Pc),qc,-A_ineq,-l_ineq,[],[],[],[],init,1e-2);

            
            ADMM_update.w = resc(1:obj.Nd*(obj.N+1),1);
            for j = 1:nnz(constraints.N_j)
                ADMM_update.w_to_j(:,actual_N_j(j)) = resc(j*obj.Nd*(obj.N+1)+1:(j+1)*obj.Nd*(obj.N+1),1);
            end
            
            
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function flag = isTunablePropertyDataTypeMutableImpl(obj)
            % Return false if tunable properties cannot change data type
            % between calls to the System object
            flag = false;
        end
        
        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end
        
        function flag = isInputDataTypeMutableImpl(obj,index)
            % Return false if input data type cannot change
            % between calls to the System object
            flag = false;
        end
        
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
            % if obj.UseOptionalOutput
            %     num = 2;
            % end
        end
        
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            %             out = [1 1];
            
            % Example: inherit size from first input port
            out = propagatedInputSize(obj,1);
        end
        
        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "Bus: ADMM";
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        
        function sts = getSampleTimeImpl(obj)
            switch char(obj.SampleTimeTypeProp)
                case 'Inherited'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'InheritedNotControllable'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'AlternatePropagation','Controllable');
                case 'InheritedErrorConstant'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'ErrorOnPropagation','Constant');
                case 'FixedInMinorStep'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete'
                    sts = createSampleTime(obj,'Type','Discrete',...
                        'SampleTime',obj.TickTime, ...
                        'OffsetTime',obj.OffsetTime);
                case 'Controllable'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end
        
        function flag = supportsMultipleInstanceImpl(obj)
            % Return true if System block can be used inside a For Each
            % subsystem, which requires multiple object instances
            flag = true;
        end
        
        function flag = allowModelReferenceDiscreteSampleTimeInheritanceImpl(obj)
            % Return true if sample time inheritance is allowed in Model
            % blocks
            flag = true;
        end
        
        
    end
end
