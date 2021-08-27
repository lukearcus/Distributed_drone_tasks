classdef predictADMM < matlab.System & matlab.system.mixin.Propagates
    % PREDICTADMM Updates the ADMM vector during the predict cycle
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
        A
        B
        Q
        R
        min_input
        max_input
        u_bound
        l_bound
        posM
        d
        posMN
        Ax
        Bu
        Aeq
        b_eq
    end
     
    properties(Access = private, Nontunable)
        Nd = 3; % number of dimensions
        nx
        nu
        umax = 1;
        umin = -1;
        admmxlength
        admmulength
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.A = [1 0 0  obj.SampleTime 0 0;
                0 1 0 0  obj.SampleTime 0;
                0 0 1 0 0  obj.SampleTime;
                0 0 0 1 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];
            
            obj.B = [0 0 0;
                0 0 0;
                0 0 0;
                obj.SampleTime 0 0;
                0  obj.SampleTime 0;
                0 0  obj.SampleTime];
            
            obj.nx = size(obj.A,1); %state vector dimension
            obj.nu = size(obj.B,2); %number of inputs
            
            obj.admmxlength = obj.nx*(obj.N+1);
            obj.admmulength = obj.nu*(obj.N);
            
            obj.Q = blkdiag(eye(obj.Nd)*10,zeros(obj.nx-obj.Nd));
            obj.R = eye(obj.nu)*13;
            
            obj.min_input = repmat(ones(obj.nu,1)*obj.umin,obj.N,1);
            obj.max_input = repmat(ones(obj.nu,1)*obj.umax,obj.N,1);
            
            obj.u_bound = [inf((obj.N+1)*obj.nx,1);obj.max_input];
            obj.l_bound = [-inf((obj.N+1)*obj.nx,1);obj.min_input];
            
            
            obj.posM = blkdiag(eye(obj.Nd),zeros(obj.nx-obj.Nd)); % Matrix to take only position from the state vector x (zeros rest)
            obj.d = [eye(obj.Nd),zeros(obj.Nd,obj.nx-obj.Nd)]; % Matrix to take only position from the state vector x (only returns pos)
            obj.posMN = kron(eye(obj.N+1),obj.d); %returns pos across time steps
            
            obj.Ax = kron(speye(obj.N+1), -speye(obj.nx)) + kron(sparse(diag(ones(obj.N, 1), -1)), obj.A);
            obj.Bu = kron([sparse(1, obj.N); speye(obj.N)], obj.B);
            obj.Aeq = [obj.Ax, obj.Bu];
            
            
        end
        
        function ADMM_update = stepImpl(obj, ADMM_input, constraints)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            ADMM_update = ADMM_input;
            
            obj.b_eq = [-constraints.x0 zeros(1, obj.N*obj.nx)]';
            
            P_new = blkdiag( kron(eye(obj.N), obj.Q+ (nnz(constraints.N_j)+1)*ADMM_input.rho/2*obj.posM ), ...
                obj.Q+(nnz(constraints.N_j)+1)*ADMM_input.rho/2*obj.posM, kron(eye(obj.N), obj.R) );
            
            q = prediction_linear(ADMM_input.lambda, ...
                ADMM_input.lambda_from_j, ...
                ADMM_input.w, ...
                ADMM_input.w_from_j, ...
                ADMM_input.rho, ...
                constraints.r', obj.Q, obj.N, obj.nu, obj.posMN,nonzeros(constraints.N_j));
            
            init = [ADMM_input.x; ADMM_input.u];
            opts = optimoptions('quadprog','Display','off','Algorithm','Active-set');
            res = quadprog(P_new,q,[],[], full(obj.Aeq), obj.b_eq, obj.l_bound, obj.u_bound,init,opts);
%             res = qp_grad(P_new,q,[],[], full(obj.Aeq), obj.b_eq, obj.l_bound, obj.u_bound,init,1e-2);
            
            ADMM_update.x = res(1:obj.admmxlength);
            ADMM_update.u = res(obj.admmxlength+1 : obj.admmxlength + obj.admmulength);
            
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
