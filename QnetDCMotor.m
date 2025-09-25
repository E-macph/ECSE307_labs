classdef QnetDCMotor < handle
    %QNETDCMOTOR Interface for Qnet DC Motor.
    %   This class implements an interface to QNET Allied Motion CL40
    %   Series Coreless DC Motor (model 16705). The motor is driven through
    %   an analogue input (voltage). The motor is equipped with three
    %   sensors: current (A), angular velocity tachometer (units/s) and
    %   angle position encoder (units).
    %   * Syntax:
    %       Motor = QnetDCMotor(); % Initialize a connection with Motor   .
    %       Motor = QnetDCMotor('Port', 'Dev1'); % Same but Use port 'dev1'
    %
    %   * Functions:                                                      .
    %       Motor.off();       % Set the power to off                     .
    %       Motor.on();        % Set the power to on                      .
    %       Motor.reset();     % Reset the power to off then on           .
    %       Motor.input(v);    % Send the voltage v                       .
    %       Motor.input(v, t); % Send the voltage v at time t             .
    %       Motor.wait(t);     % Wait until the Motor internal timer is t.
    %
    %   * Properties
    %       time = Motor.time;         % Get time (s)                     .
    %       voltage = Motor.voltage;   % Get voltage (V)                  .
    %       current = Motor.current;   % Get electric current (A)         .
    %       velocity = Motor.velocity; % Get angular velocity (units/s)   .
    %       angle = Motor.angle;       % Get current angle (units)        .
    %
    %   * Other usage:
    %      Configurable properties are:
    %        - MaxVoltage: Maximum voltage to send to DC motor            .
    %        - MinVoltage: Minimum voltage to send to DC motor            .
    %        - Units: Determines the units of the angle, use 'deg' for
    %        degrees or 'rad' for radians                                 .
    %        - BufferSize: Buffer size in seconds, specifies the amount of
    %        data saved internally by the motor                           .
    %
    %      It is possible to set the sampling time with the function
    %      <setSamplingTime>.
    %
    %      The <input> function supports an array voltage as argument. If
    %      an array of voltage is requested, values of this voltage will be
    %      sent at each sampling time. You can also specify 2 arguments, an
    %      array of voltage and an array of time, in this case input will
    %      send each value in the array of voltage at the corresponding
    %      index in the time array.
    %
    %   Author: anas.elfathi@mcgill.ca.                       August 2017.
    %   Last revision: 06-February-2019.
    
    properties(GetAccess = public, SetAccess = public)
        MaxVoltage = 7.0; % Maximum voltage to send to DC motor.
        MinVoltage = -7.0; % Minimum voltage to send to DC motor.
        Units = 'rad'; % Determines the units of the angle, use 'deg' for degrees or 'rad' for radians.
        BufferSize = 10 * 60; % Buffer size in seconds, changes in buffer size are only effective after a motor RESET.
    end
    
    properties(GetAccess = public, SetAccess = private)
        Power = false; % DC motor Power. use ON and OFF to set and reset Motor.
        DevicePort = 'Dev1'; % Device port where the DC Motor is connected. To set a specific port do <QnetDCMotor('port', 'dev#');> where # is the port number.
        SamplingTime = 0.01; % Sampling time (seconds), use SETSAMPLINGTIME to modify sampling time.
        IsConnected = true; % Flag to indicate that the DC Motor is connected
    end
    
    properties(Dependent)
        time % Get history of time (sec).
        voltage % Get history of applied voltage (V).
        current % Get history of current (A).
        velocity % Get history of angular velocity (rad/s or deg/s).). Use <Units> to set units.
        angle % Get history of angle (rad or deg). Use <Units> to set units.
    end
    
    methods
        function val = get.time(this)
            val = this.TimeData(this.TimeData > this.StartTime-0.5*this.SamplingTime) - this.StartTime;
        end
        
        function val = get.angle(this)
            val = this.AngleData(this.TimeData > this.StartTime-0.5*this.SamplingTime) * (180 / pi)^strcmpi(this.Units, 'deg');
        end
        
        function val = get.velocity(this)
            val = this.SpeedData(this.TimeData > this.StartTime-0.5*this.SamplingTime) * (180 / pi)^strcmpi(this.Units, 'deg');
        end
        
        function val = get.current(this)
            val = this.CurrentData(this.TimeData > this.StartTime-0.5*this.SamplingTime);
        end
        
        function val = get.voltage(this)
            val = this.VoltageData(this.TimeData > this.StartTime-0.5*this.SamplingTime);
        end
    end
    
    properties(GetAccess = private, SetAccess = private)
        % Data buffers
        TimeData = [];
        VoltageData = [];
        CurrentData = [];
        SpeedData = [];
        AngleData = [];
        
        % Recovery parameters
        RecoveryFileName = 'dataQnetMotor.mat';
        
        % Scope parameters
        ScopeFig = [];
        ScopeTimer = [];
        ScopeAxes = [];
        
        StartTime = []; % Holds the time when first drive command is issued, internal variable used to sync the DC Motor time
        
        Sess = []; % Structure for data acquisition sessions
        Voltage = 0; % Last changed voltage
        
        Dist = []; % Structure for disturbance
    end
    
    methods(Access = public)
        function this = QnetDCMotor(varargin)
            %QNETDCMOTOR [INTERFACE] Creates a QnetDCMotor object.
            % set up all Data Acquisition sessions, set up default sampling
            % time, set up scope parameter.
            % * Optional parameter:                                       .
            %    - 'port': Device port where the DC Motor is connected    .
            %    - 'recovery': Default recovery file name                 .
            %
            
            % Default Device port
            try
                devices = daq.getDevices;
                this.DevicePort = devices(1).ID;
            catch
                this.DevicePort = 'Dev1';
            end
            
            % Get options
            for nVar = 1:2:length(varargin)
                switch lower(varargin{nVar})
                    case 'port'
                        this.DevicePort = varargin{nVar+1};
                    case 'recovery'
                        this.RecoveryFileName = varargin{nVar+1};
                    otherwise
                        error('QnetDCMotor::WrongInput::Unknown option %s, use doc QnetDCMotor for more information.', varargin{nVar});
                end
            end
            
            % Initialize monitoring utilities
            this.createFigure();
            this.createScope();
            
            try
                % Initialize Motor Connection
                this.setUpChannels();
                
                % set default Sampling Time
                this.setSamplingTime(this.SamplingTime);
                
                % set on
                this.on();
            catch expect
                fprintf(2, 'QnetDCMotor::Constructor::Error:: Qnet DC Motor is not connected or already in use.\n * Make sure Qnet DC Motor is connected via the USB port.\n * Make sure you are using a valid device ID, use <help QnetDCMotor.DevicePort> for more information.\n * If you are using Qnet DC Motor in another script, clear the motor object, then call <daq.reset>.\n * If you are using Qnet DC Motor in an external program exit that program.\n If nothing works restart MATLAB!\n')
                this.Sess = [];
                this.IsConnected = false;
                if exist(this.RecoveryFileName, 'file') == 2
                    fprintf('QnetDCMotor::Constructor::Info:: Entering recovery mode with file %s.\n', this.RecoveryFileName);
                else
                    throw(expect);
                end
            end
        end
        
        function delete(this)
            %DELETE [INTERFACE] Safely disconnect motor.
            if ~isempty(this.Sess)
                % reset drive
                if isvalid(this.Sess.drive)
                    this.Sess.drive.outputSingleScan(0);
                    pause(0.2);
                    this.Sess.drive.stop;
                    this.Sess.drive.release;
                end
                
                % reste power
                if isvalid(this.Sess.power)
                    this.Sess.power.outputSingleScan([0, 1]);
                    pause(0.5);
                    this.Sess.power.stop;
                    this.Sess.power.release;
                end
                
                %reset monitor
                if isvalid(this.Sess.monitor)
                    this.Sess.monitor.stop;
                    this.Sess.monitor.release;
                end
                
                this.Sess = [];
            end
        end
        
        function setSamplingTime(this, dt_)
            %SETSAMPLINGTIME [INTERFACE] Set the sampling time (seconds).
            this.SamplingTime = dt_;
            
            prevStatus = this.Power;
            this.off;
            
            % reset monitoring Rate
            this.Sess.monitor.Rate = 1 / dt_; % Hz
            this.Sess.monitor.NotifyWhenDataAvailableExceeds = 1;
            if this.Sess.monitor.NotifyWhenDataAvailableExceeds > this.BufferSize
                warning('QnetDCMotor::setSamplingTime::Warning::Insufficient BufferSize %d for the request Sampling time %d. Buffer size increased to %d.', ...
                    this.BufferSize, ...
                    dt_, ...
                    this.Sess.monitor.NotifyWhenDataAvailableExceeds);
                this.BufferSize = this.Sess.monitor.NotifyWhenDataAvailableExceeds;
            end
            
            % reset driving Rate
            this.Sess.drive.Rate = 1 / dt_; % Hz
            this.Sess.drive.NotifyWhenScansQueuedBelow = 1;
            
            if prevStatus
                this.on;
            end
        end
        
        function on(this)
            %ON [INTERFACE] Set ON DC motor.
            if ~this.IsConnected
                return
            end
            
            if ~this.Power
                % reset internal buffers
                this.resetBuffers();
                
                % restart motor
                this.Sess.power.outputSingleScan([1, 0]);
                pause(0.2);
                
                % restart acquiring data
                this.Sess.monitor.startBackground();
                pause(1.0);
                
                % reset real time counter
                this.StartTime = this.TimeData(end);
                
                % set power flag to on
                this.Power = true;
            end
        end
        
        function off(this)
            %OFF [INTERFACE] Set OFF DC motor.
            if ~this.IsConnected
                return
            end
            
            if this.Power
                % set disturbance to empty
                this.Dist = [];
                
                % reset motor
                this.input(0);
                pause(0.2);
                
                this.Sess.power.outputSingleScan([0, 1]);
                pause(0.2);
                
                % stop acquiring data
                this.Sess.monitor.stop();
                pause(0.5);
                
                % set power flag to off
                this.Power = false;
            end
        end
        
        function reset(this)
            %RESET [INTERFACE] Reset DC motor.
            % Clear all buffers, power off and on the DC motor, reset DC
            % motor internal timer.
            if ~this.IsConnected
                return
            end
            
            this.off();
            this.on();
        end
        
        function disturbance(this, time, value)
            %DISTURBANCE [INTERFACE] Program a speed disturbance for DC
            %motor.
            % * Inputs:
            %  - time  : Time interval when the disturbance is applied. -
            %  value : Time (sec) when the voltage is applied. if time
            if ~this.IsConnected || ~this.Power
                return
            end
            
            time = time(:);
            value = value(:);
            if any(size(time) ~= size(value))
                error('QnetDCMotor::Disturbance::Error: time and value should have the same size!');
            end
            this.Dist.Time = (time(1):this.SamplingTime:time(end))';
            this.Dist.Value = pchip(time, value, this.Dist.Time) / 28.5;
        end
        
        function wait(this, time)
            %WAIT [INTERFACE] Sync the motor internal timer with the
            % specified time in (seconds). the internal timer of the motor
            % is counted from the moment the motor is on.
            
            if ~this.IsConnected || ~this.Power
                return
            end
            
            % Sleep until internal time is synced with time
            while this.TimeData(end) - this.StartTime - time + 0.5 * this.SamplingTime < 0
                pause(this.SamplingTime/10);
            end
        end
        
        function input(this, voltage, time)
            %INPUT [INTERFACE] Drives DC motor
            % Drives DC motor with "voltage" at "time"
            % * Inputs:
            %  - voltage: Voltage in (V) to be sent to DC motor. If voltage
            %  is an array, input will send each value in this array, one
            %  at a time.
            %
            %  - time : Optional. specifies the time when the voltage will
            %  be sent. time should have the same size as voltage.
            %
            % * Syntax:
            %   Motor = QnetDCMotor();                                   .
            %   Motor.input(3);     % Send 3 volts                       .
            %   Motor.input(0, 10); % Send 0 volts at time 10s           .
            %
            
            if ~this.IsConnected || ~this.Power
                return
            end
            
            if ~isscalar(voltage)
                if nargin < 3
                    time = this.time(end) + ...
                        0:this.SamplingTime:(length(voltage) - 1) * this.SamplingTime;
                end
                
                for k = 1:1:length(time)
                    this.input(voltage(k), time(k));
                end
            else
                if voltage > this.MaxVoltage
                    warning('QnetDCMotor::drive::Warning: Value saturated to %fV', this.MaxVoltage);
                end
                if voltage < this.MinVoltage
                    warning('QnetDCMotor::drive::Warning: Value saturated to %fV', this.MinVoltage);
                end
                
                % saturate voltage
                voltage = min(this.MaxVoltage, voltage);
                voltage = max(this.MinVoltage, voltage);
                
                % update the voltage command
                this.Voltage = voltage;
                if ~isempty(this.Dist)
                    idx = floor((this.TimeData(end) - this.StartTime - this.Dist.Time(1))/this.SamplingTime) + 1;
                    if idx >= 1 && idx < length(this.Dist.Time)
                        voltage = voltage + this.Dist.Value(idx);
                    end
                end
                
                if nargin == 3
                    this.wait(time);
                end
                
                this.Sess.drive.outputSingleScan(voltage);
            end
        end
        
        function scope(this, bufferTime)
            %SCOPE [INTERFACE] Opens a figure to monitor the DC motor.
            % Type 'p' if you want to take a snapshot of the scope or 'ESC'
            % to close the scope.
            
            if nargin < 2
                bufferTime = min(30, this.BufferSize); % default 30 seconds
            end
            if ~this.ScopeFig.isvalid
                this.createFigure();
            end
            figure(this.ScopeFig);
            this.ScopeAxes.Voltage = subplot(2, 2, 1);
            this.ScopeAxes.Velocity = subplot(2, 2, 2);
            this.ScopeAxes.Current = subplot(2, 2, 3);
            this.ScopeAxes.Angle = subplot(2, 2, 4);
            if ~isvalid(this.ScopeTimer)
                this.createScope();
            end
            stop(this.ScopeTimer);
            this.ScopeTimer.TimerFcn = @(src, event)this.updateScope(src, event, bufferTime);
            start(this.ScopeTimer);
        end
        
        function ok = load(this, filename, tag)
            %LOAD [INTERFACE] Load data with the name tag from filename
            
            if nargin < 3
                tag = filename;
                filename = this.RecoveryFileName;
            end
            
            ok = 0;
            
            % remove extension and path
            [~, filename, ~] = fileparts(filename);
            
            if exist([filename, '.mat'], 'file') == 0
                warning('QnetDCMotor::load::Warning: Recovery file %s not found.', filename)
                return
            end
            
            if this.IsConnected
                this.off;
            end
            
            tag = regexprep(tag, '[\s\.-]{1}', '_', 'all');
            
            
            eval(['load(filename, ''', tag, ''');']);
            try
                eval(['data = ', tag, ';']);
            catch
                warning('QnetDCMotor::load::Warning: Could not load data %s from Recovery file %s.', tag, filename)
                return
            end
            
            ids = nan(1, numel(data.Students));
            for i = 1:numel(data.Students)
                ids(i) = hex2dec(data.Students(i).ID) / 7907;
            end
            fprintf(['QnetDCMotor::load::info: Recovering data collected by students: ', repmat('[%d] ', size(ids)), '.\n'], ids);
            
            this.StartTime = data.StartTime;
            this.TimeData = data.TimeData;
            this.VoltageData = data.VoltageData;
            this.CurrentData = data.CurrentData;
            this.SpeedData = data.SpeedData;
            this.AngleData = data.AngleData;
            
            ok = 1;
        end
        
        function save(this, students, filename, tag)
            %SAVE [INTERFACE] Save data with the name tag in filename
            
            if ~this.IsConnected
                return
            end
            
            if nargin < 4
                tag = filename;
                filename = this.RecoveryFileName;
            end
            
            % remove extension and path
            [~, filename, ~] = fileparts(filename);
            
            tag = regexprep(tag, '[\s\.-]{1}', '_', 'all');
            
            data.StartTime = this.StartTime;
            data.TimeData = this.TimeData;
            data.VoltageData = this.VoltageData;
            data.CurrentData = this.CurrentData;
            data.SpeedData = this.SpeedData;
            data.AngleData = this.AngleData;
            for i = 1:numel(students)
                data.Students(i).ID = dec2hex(str2double(students(i).ID)*7907);
            end
            
            eval([tag, '= data;']);
            
            if exist([filename, '.mat'], 'file') == 0
                eval(['save(filename, ''', tag, ''');']);
            else
                eval(['save(filename, ''', tag, ''', ''-append'');']);
            end
        end
    end
    
    methods(Access = private)
        function setUpChannels(this)
            %SETUPCHANNELS create all sessions and connect appropriate
            % reset any previous connection
            daq.reset;
            
            %channels
            this.Sess.monitor = daq.createSession('ni');
            this.Sess.monitor.IsContinuous = true;
            % input channel to read MotorEncoder
            this.Sess.monitor.addCounterInputChannel(this.DevicePort, 'ctr0', 'Position');
            this.Sess.monitor.Channels(end).EncoderType = 'X4';
            this.Sess.monitor.Channels(end).ZResetEnable = true;
            this.Sess.monitor.Channels(end).ZResetValue = 2048;
            % input channel to read Current
            this.Sess.monitor.addAnalogInputChannel(this.DevicePort, 'ai0', 'Voltage');
            % input channel to read Velocity
            this.Sess.monitor.addAnalogInputChannel(this.DevicePort, 'ai4', 'Voltage');
            % add listener to acquire data
            this.Sess.monitor.addlistener('DataAvailable', @this.updateMeasurement);
            
            this.Sess.drive = daq.createSession('ni');
            % output channel to send voltage command
            this.Sess.drive.addAnalogOutputChannel(this.DevicePort, 'ao0', 'Voltage');
            
            this.Sess.power = daq.createSession('ni');
            this.Sess.power.Rate = 10;
            % output channel to enable/disable amplifier
            this.Sess.power.addDigitalChannel(this.DevicePort, 'Port0/Line21:22', 'OutputOnly');
        end
        
        function updateMeasurement(this, src, event)
            %%UPDATEMEASUREMENT Callback to update local buffers from DC
            %%motor
            % sensors
            
            % cycle
            if event.TimeStamps(end) > this.TimeData(end)
                this.TimeData(1:end-src.NotifyWhenDataAvailableExceeds) = this.TimeData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.VoltageData(1:end-src.NotifyWhenDataAvailableExceeds) = this.VoltageData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.CurrentData(1:end-src.NotifyWhenDataAvailableExceeds) = this.CurrentData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.SpeedData(1:end-src.NotifyWhenDataAvailableExceeds) = this.SpeedData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.AngleData(1:end-src.NotifyWhenDataAvailableExceeds) = this.AngleData(src.NotifyWhenDataAvailableExceeds+1:end);
            end
            
            % update last data
            this.TimeData(end-src.NotifyWhenDataAvailableExceeds+1:end) = event.TimeStamps;
            this.VoltageData(end-src.NotifyWhenDataAvailableExceeds+1:end) = this.Voltage * ones(size(event.TimeStamps));
            this.CurrentData(end-src.NotifyWhenDataAvailableExceeds+1:end) = event.Data(:, 2) * 0.59;
            this.SpeedData(end-src.NotifyWhenDataAvailableExceeds+1:end) = event.Data(:, 3) * 78;
            unsignedAngle = event.Data(:, 1);
            unsignedAngle(unsignedAngle > 2^31) = unsignedAngle(unsignedAngle > 2^31) - 2^32;
            this.AngleData(end-src.NotifyWhenDataAvailableExceeds+1:end) = unsignedAngle * (pi / 1024);
        end
        
        function resetBuffers(this)
            %RESETBUFFERS reset internal buffer and set time to 0.
            this.TimeData = nan(ceil(this.BufferSize/this.SamplingTime), 1);
            this.VoltageData = nan(ceil(this.BufferSize/this.SamplingTime), 1);
            this.CurrentData = nan(ceil(this.BufferSize/this.SamplingTime), 1);
            this.SpeedData = nan(ceil(this.BufferSize/this.SamplingTime), 1);
            this.AngleData = nan(ceil(this.BufferSize/this.SamplingTime), 1);
        end
        
        function createFigure(this)
            %CREATEFIGURE create figure for scope
            this.ScopeFig = findobj(groot, 'Number', 7070707);
            if isempty(this.ScopeFig)
                this.ScopeFig = figure(7070707);
            end
            this.ScopeFig.Visible = 'off';
            clf(this.ScopeFig);
            this.ScopeFig.Name = 'DC Motor Monitor';
            this.ScopeFig.KeyPressFcn = 'set(gcbf,''UserData'',double(get(gcbf,''Currentcharacter''))) ; uiresume ';
            this.ScopeFig.NumberTitle = 'off';
        end
        
        function createScope(this)
            %CREATESCOPE create scope
            
            % clean up all previous scopes, if any.
            for t = timerfind('Name', 'DC-Motor-Monitor')
                stop(t);
                delete(t);
            end
            % set up scope parameter
            this.ScopeTimer = timer;
            this.ScopeTimer.Period = 0.2;
            this.ScopeTimer.ExecutionMode = 'fixedRate';
            this.ScopeTimer.Name = 'DC-Motor-Monitor';
        end
        
        function updateScope(this, src, ~, bufferTime)
            %UPDATESCOPE Callback to update the scope plot
            if ~this.ScopeFig.isvalid
                stop(src)
            elseif ~isempty(this.ScopeFig.UserData)
                switch this.ScopeFig.UserData
                    case 27 % ESC key
                        this.ScopeFig.Visible = 'off';
                        stop(src)
                    case 'p'
                        savefig(this.ScopeFig, ['Snapshot_DC_Motor_', datestr(now, 'yymmmdd_HHMMSS')]);
                    otherwise
                        fprintf('Pressed <%s> ... Use ''p'' to save a snapshot of the figure, ''ESC'' to close monitor\n', char(this.ScopeFig.UserData))
                end
                this.ScopeFig.UserData = [];
            else
                idx = find(this.TimeData >= bufferTime*floor(this.TimeData(end)/bufferTime), 1);
                if ~isempty(idx)
                    plot(this.ScopeAxes.Voltage, this.TimeData(idx:end), this.VoltageData(idx:end))
                    xlim(this.ScopeAxes.Voltage, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0, 1]))
                    
                    plot(this.ScopeAxes.Current, this.TimeData(idx:end), this.CurrentData(idx:end))
                    xlim(this.ScopeAxes.Current, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0, 1]))
                    
                    plot(this.ScopeAxes.Velocity, this.TimeData(idx:end), this.SpeedData(idx:end)*(180 / pi)^strcmpi(this.Units, 'deg'))
                    xlim(this.ScopeAxes.Velocity, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0, 1]))
                    grid(this.ScopeAxes.Velocity, 'on');
                    title(this.ScopeAxes.Velocity, sprintf('Angular Velocity (%s/s)', this.Units))
                    
                    plot(this.ScopeAxes.Angle, this.TimeData(idx:end), this.AngleData(idx:end)*(180 / pi)^strcmpi(this.Units, 'deg'))
                    xlim(this.ScopeAxes.Angle, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0, 1]))
                end
                grid(this.ScopeAxes.Voltage, 'on');
                title(this.ScopeAxes.Voltage, 'Voltage (V)')
                grid(this.ScopeAxes.Current, 'on');
                title(this.ScopeAxes.Current, 'Current (A)')
                grid(this.ScopeAxes.Angle, 'on');
                title(this.ScopeAxes.Angle, sprintf('Angle (%s)', this.Units))
            end
        end
    end
end
