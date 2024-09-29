classdef myTriax < handle
    % Matlab class to control Horiba Jobin-Yvon TRIAX monochromators.
    %
    % It uses Serial Port or GPIB to comunicate with the device.
    % Serial port can be from COM1 up to COM256. However, Horiba software 'Hardware Configuration and Control' is
    % limited to works with serial port from COM1 up to COM4.
    % Grating Parameters and Slit Parameters are provided in the class properties below and in the myTriax() function.
    % Some Triax monochromators develop problems changing gratings. To solve this problem, motor speed can be decreased
    % to increase motor torque (HIGHMOTORTORQUEFLAG = true).
    %
    % Instructions:
    % mono = myTriax('Triax 320');                                  % Create device object.
    % mono.connect('SerialPort','Port','COM1','BaudRate',19200);    % Connect and start device.
    % mono.initializeMotors();                                      % Initialize motors if object was not initialized before.
    % mono.exitMirrorFront();                                       % Select front exit.
    % mono.moveGratingIndex(2);                                     % Select grating.
    % mono.moveSlitWidth(0,'max');                                  % Open front entrace slit to maximum width.
    % mono.moveSlitWidth(2,'max');                                  % Open front exit slit to maximum width.
    % mono.moveSlitWidth(3,0);                                      % Close lateral exist slit.
    % mono.moveTo(0);                                               % Change wavelength.
    % mono.disconnect();                                            % Disconnect device.
    %
    % Author: F.O.
    % Created: 2022
    % Update: 2023-07-28    - Correction to 'moveGratingIndex(h,val)'
    
    properties (Constant, Hidden)
       % Time parameters 
       TIMEOUT             = 1;               	% Default timeout time (seconds) for settings change.
       TIMEREAD            = 0.05;             	% Default time (seconds) to read portObject.
       TIMEINITIALIZE      = 80;               	% Minimum time (seconds) necessary to initialize monochromator.
       TIMEMOVEMIRROR      = 2;                 % Time to move entrande and exit mirrors.
       TIMEMOVESHUTTER     = 0.1;               % Time to open / close shutter.
       
       % Turrent Gratings Gr/mm
       BASEGRATING = 1200;                      % Grating density of groves base value.
       GRATING_0   = 1200;                     	% Grating density of groves, turret index position 0.
       GRATING_1   = 900;                     	% Grating density of groves, turret index position 1.
       GRATING_2   = 600;                    	% Grating density of groves, turret index position 2.
       
       % Turrent Rough Calibration nm
       GRATINGCALIBARTION_0 = -10;              % Grating rough calibration, turrent index position 0.
       GRATINGCALIBARTION_1 = -200;             % Grating rough calibration, turrent index position 0.
       GRATINGCALIBARTION_2 = -300;             % Grating rough calibration, turrent index position 0.
       
       % Turrent Motor Speed & Rise Time Limits
       GRATINGMOTORMINSPEED    = 100;           % Minimum frequency in Hz (steps per second) provided in the manual for all monochromators (typical for a triax = 1000 Hz).
       GRATINGMOTORMAXSPEED    = 80000;         % Maximum frequency in Hz (steps per second) provided in the manual for all monochromators (typical for a triax = 4000 Hz).
       GRATINGMOTORMINRISETIME = 100;           % Minimum rise time (is milisecond) provided in the manual for all monochromators (typical rise time for a triax = 250 ms).
       GRATINGMOTORMAXRISETIME = 65535;         % Maximum rise time (is milisecond) provided in the manual for all monochromators (typical rise time for a triax = 250 ms).
       
       % Turrurent Motor High Torque Speed - used if monochromator has difficuty changing diffraction garting
       HIGHMOTORTORQUEFLAG  = true;            	% Change speed motor to minimum speed to increase torque while changing gratings.
       HIGHMOTORTORQUESPEED = '1000,1000,250';	% Minimum speed for grating motor Horiba monochromator Triax 180, 190 and 320.
                                               	% Monochromator System = 0; Min. frequency = 1000 steps/sec.; Max. frequency = 1000 steps/sec.; Ramp time = 250 miliseconds.
       % Slit Parameters
       SLITAPERTUREMAX        = 2.24;          	% Slit maximum aperture 2.24 mm.
       SLITMOTORSPEED         = 300;            % Default slit motor frequency/speed in Hz (steps per second).
       SLITMOTORSTEPSMAX      = 1080;          	% Slit, number of steps between minimum and maximum aperture.
       SLITMOTORSTEPSBACKLASH = 10;             % Slit motor backlash in motor steps.
       SLITCONVERTIONFACTOR   = 482.14;      	% Convertion factor: motor steps per milimeter.
       
       % Slit Motor Speed Limits
       SLITMOTORMINSPEED      = 10;             % Slit motor minimum speed in Hz (steps per second).
       SLITMOTORMAXSPEED      = 10000;          % Slit motor maximum speed in Hz (steps per second).
       
       % Error Messages
       MSGERROR   = 'Error: ';
       MSGWARNING = 'Warning: ';
       CLASSNAME  = 'myTriax.m > ';
    end
    
    properties (Hidden)
       % Triax Parameters Adjusted according to Model by function myTriax(model)
       SPECTRALDISPERSION   = 2.64;             % Spectral dispersion (base grating: 1200 gr/mm): Triax 180/190 = 3.53nm, Triax 180/190 = 2.64nm, Triax 180/190 = 1.55nm.
       GRATINGMOTORMINLIMIT = -200;             % Minimum wavelength limit ((base grating: 1200 gr/mm): Triax 180/190 = -100nm, Triax 320/550 = -200nm.
       GRATINGMOTORMAXLIMIT = 1500;             % Maximum wavelength limit (base grating: 1200 gr/mm): Triax 180/190 = 1400nm, Triax 320/550 = 1500nm.
    end

    properties 
       % These properties are within Matlab wrapper
       deviceInfo;                      % Device parameters.
       portObject;                      % Device port object: serial port ID or gpib ID.
    end

    properties (Hidden)
       % These are properties within the .NET environment.
       
       errorDetected = false;           % Flag for detected errors during execution.
       isconnected = false;             % Flag set if device connected.
       isinitialized;                   % Flag set if device connected.
       gratingIndex;                    % Set or Get grating turret index position.
       gratingBaseFactor;               % Set or Get grating base factor to correct input wavelengths.
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M E T H O D S - CONSTRUCTOR/DESCTRUCTOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods

        % =================================================================
        % FUNCTION myTriax
        % 
        function h = myTriax(model) % Constructor
            
            functionName = 'myTriax(model)';    % Function name string to use in messages.
            h.errorDetected = false;            % Reset errorDetected flag.
            
            % Organizing Input Information                
            switch lower(model)
                case {'triax180' 'triax 180' 'triax190' 'triax 190'}
                    h.SPECTRALDISPERSION   = 3.53;              % Spectral dispersion (base grating: 1200 gr/mm).
                    h.GRATINGMOTORMINLIMIT = -100;              % Minimum wavelength limit (base grating: 1200 gr/mm).
                    h.GRATINGMOTORMAXLIMIT = 1400;              % Maximum wavelength limit (base grating: 1200 gr/mm).

                case {'triax320' 'triax 320'}
                    h.SPECTRALDISPERSION   = 2.64;              % Spectral dispersion (base grating: 1200 gr/mm).
                    h.GRATINGMOTORMINLIMIT = -200;              % Minimum wavelength limit (base grating: 1200 gr/mm).
                    h.GRATINGMOTORMAXLIMIT = 1500;              % Maximum wavelength limit (base grating: 1200 gr/mm).

                case {'triax550' 'triax 550'}
                    h.SPECTRALDISPERSION   = 1.55;              % Spectral dispersion (base grating: 1200 gr/mm).
                    h.GRATINGMOTORMINLIMIT = -200;              % Minimum wavelength limit (base grating: 1200 gr/mm).
                    h.GRATINGMOTORMAXLIMIT = 1500;              % Maximum wavelength limit (base grating: 1200 gr/mm).

                otherwise
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': monochromator model invalid.']);
                    return;
            end
            
        end

        % =================================================================
        function delete(h) % Destructor 
            
            if ~isempty(h.portObject) && h.isconnected
                disconnect(h);
            end
        end

    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M E T H O D S (Sealed) - INTERFACE IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods (Sealed)

        % =================================================================
        % FUNCTION: CONNECT MONOCHROMATOR
        function isinitialized = connect(h,varargin)
            
            functionName    = 'connect()';      % Function name string to use in messages.
            h.errorDetected = false;            % Reset errorDetected flag.
            isinitialized   = true;             % Initialization is required only after monochromator is power-up.
                      
            if ~h.isconnected                       % Execute only if it was not initilaed previously.
                
                % Device Information for Connection
                h.deviceInfo.portChoice          = char.empty;          % Port for connection: serial port or gpib.
                h.deviceInfo.serialPort.port     = char.empty;          % Serial port name: 'COM1', 'COM2',...
                h.deviceInfo.serialPort.baudRate = 19200;               % Serial port baudrate: '9600', '19200',...
                                                                        % All other serial port properties are fixed by the device.
                h.deviceInfo.gpib.vendor         = char.empty;          % GPIB vendor name: 'ni', 'keysight',...
                h.deviceInfo.gpib.boardindex     = uint8.empty;         % GPIB board index.
                h.deviceInfo.gpib.primaryAddress = 1;                   % GPIB monochromator primary address.
                                                                        % monochromator does not support secondary address.
                % Organizing Input Information
                for i = 1:(nargin - 1)
                    
                    switch lower(varargin{i})
                        case {'serialport' 'serial port'}
                            h.deviceInfo.portChoice = 'serialport';

                        case 'gpib'
                            h.deviceInfo.portChoice = 'gpib';

                        case 'port'
                            i = i+1;

                            % Verify Validity of the Input Value
                            if contains(lower(varargin{i}),'com')
                                
                                vararginSize = size(varargin{i},2);     % Serial port number 'COMxx' can have 1 or 2 digits.
                                
                                if vararginSize == 4 || vararginSize == 5  % 'COM' + 1 or 2 digit number.
                                        h.deviceInfo.serialPort.port = varargin{i};
                                else
                                    h.errorDetected = true;
                                    disp([h.MSGERROR h.CLASSNAME functionName ': Serial port number invalid.']);
                                    return
                                end

                            end

                            % If port choise is empty, adds port choise
                            if isempty(h.deviceInfo.portChoice)
                                h.deviceInfo.portChoice = 'serialport';
                            end
                            
                        case 'baudrate'
                            i = i+1;

                            % Verify Validity of the Input Value
                            if isnumeric(varargin{i})
                                switch varargin{i}
                                    case {1200 2400 4800 9600 19200}        % All option available for Triax monochromators
                                        h.deviceInfo.serialPort.baudRate = varargin{i};
                                    otherwise
                                        h.errorDetected = true;
                                        disp([h.MSGERROR h.CLASSNAME functionName ': Serial port baudrate invalid.']);
                                        return
                                end
                            else
                                h.errorDetected = true;
                                disp([h.MSGERROR h.CLASSNAME functionName ': Serial port baudrate invalid.']);
                                return
                            end

                            % If port choise is empty, adds port choise
                            if isempty(h.deviceInfo.portChoice)
                                h.deviceInfo.portChoice = 'serialport';
                            end
                            
                        case 'vendor'
                            i = i+1;

                            % Verify Validity of the Input Value - All Vendors Available in Matlab
                            if contains(lower(varargin{i}),'keysight')
                                h.deviceInfo.gpib.vendor = 'keysight';
                            elseif contains(lower(varargin{i}),'ics')
                                h.deviceInfo.gpib.vendor = 'ics';
                            elseif contains(lower(varargin{i}),'mcc')
                                h.deviceInfo.gpib.vendor = 'mcc';
                            elseif contains(lower(varargin{i}),'ni')
                                h.deviceInfo.gpib.vendor = 'ni';
                            elseif contains(lower(varargin{i}),'adlink')
                                h.deviceInfo.gpib.vendor = 'adlink';
                            else
                                h.errorDetected = true;
                                disp([h.MSGERROR h.CLASSNAME functionName ': GPIB vendor invalid.']);
                                return
                            end

                            % If port choise is empty, adds port choise
                            if isempty(h.deviceInfo.portChoice)
                                h.deviceInfo.portChoice = 'gpib';
                            end
                            
                        case {'boardindex' 'board index'}
                            i = i+1;

                            % Verify Validity of the Input Value
                            if isscalar(varargin{i}) && varargin{i} >= 0 && varargin{i} <= 99
                                h.deviceInfo.gpib.boardindex = varargin{i};
                            else
                                h.errorDetected = true;
                                disp([h.MSGERROR h.CLASSNAME functionName 'GPIB boardindex invalid.']);
                                return
                            end

                            % If port choise is empty, adds port choise
                            if isempty(h.deviceInfo.portChoice)
                                h.deviceInfo.portChoice = 'gpib';
                            end
                            
                        case {'primaryaddress' 'primary address'}
                            i = i+1;

                            % Verify Validity of the Input Value - All GPIB Allowed by Triax Monochromators
                            if isscalar(varargin{i}) && varargin{i} >= 1 && varargin{i} <= 31
                                h.deviceInfo.gpib.primaryAddress = varargin{i};
                            else
                                h.errorDetected = true;
                               disp([h.MSGERROR h.CLASSNAME functionName ': GPIB primary address is invalid.']);
                                return
                            end

                            % If port choise is empty, adds port choise
                            if isempty(h.deviceInfo.portChoice)
                                h.deviceInfo.portChoice = 'gpib';
                            end
                    end
                end
                
                % Connecting with Monochromator
                switch lower(h.deviceInfo.portChoice)               % Choose connection through serial port or GPIB
                    case {'serialport' 'serial port'}
                        port     = h.deviceInfo.serialPort.port;        % Serial port COM.
                        baudRate = h.deviceInfo.serialPort.baudRate;    % Serial port baudrate (monochromator recognizes baudRate during initialization).
                        dataBits = 8;                                   % Serial port dataBits, fixed by the maker.
                        stopBits = 1;                                   % Serial port stopBits, fixed by the maker.
                        parity   = 'none';                              % Serial port parity, fixed by the maker.
                        
                        h.portObject = serial(port,...                  % Create serial port object.
                                            'Baudrate',baudRate,...
                                            'DataBits',dataBits,...
                                            'StopBits',stopBits,...
                                            'Parity',parity,...
                                            'Terminator','',...         % Some commands have termination 'CR others don't have termination.
                                            'TimeOut',h.TIMEOUT);
                                        
                        flushoutput(h.portObject);                      % Remove data from port output buffer.
                        flushinput(h.portObject);                       % Remove data from port input buffer.
                        fopen(h.portObject);                            % Open the serial port connection.
                        
                        % Find Monochromator Status
                        answer = '';                                    % Initialize variable.
                        for i = 1:10                                    % When powered-up, the monochromator could need several tries to detect serial port baundrate (in general work in the second try).
                            fwrite(h.portObject,' ');                   % Send <space> command: Utility command 'Where Am I'.
                            answer = fread(h.portObject,1);             % Reads 1st character send by the monochromator.
                            pause(h.TIMEREAD);                          % Waits for other characters.
                            flushinput(h.portObject);                   % flush buffer to remove characters.
                            
                            if ~isempty(answer)
                                if answer == '*' || answer == 'B' || answer == 'F' 
                                    break;
                                elseif answer(1) == ' '                     % if anwer if <space> character, monochromator is in terminal mode.
                                    fwrite(h.portObject,248);               % Send <248> binary character command: Utility command 'Set Intelligent Mode'.
                                    pause(0.2);                             % Recomended wait time provided in the manual.
                                    fwrite(h.portObject,222);               % Send <222> binary character command: Utility command 'Re-boot If Hung'.
                                    pause(0.2);                             % Recomended wait time provided in the manual.
                                end
                            end
                            
                            if i == 10                                  % Something is not working
                                h.errorDetected = true;                 % error detected
                                disp([h.MSGERROR h.CLASSNAME functionName ': Device did not entre in remote mode.']);
                                disp('   Try to power-down and power-up device.');
                                disp('   If error persists, use original software to start device.');
                                return
                            end
                        end

                        % First communication after its is power-up or bootstrap
                        if answer(1) == '*'                     % Autobaunding was successful for the first time.
                            h.isinitialized = false;            % Class flag to know if monochromator is initialized.
                            isinitialized   = false;            % Output function flag to know if monochromator is initialized.
                            fwrite(h.portObject,247);           % Send <247> binary character command: Utility command 'Startup Intelligent Mode'.
                            answer = fread(h.portObject,1);     % Reads answer, it should be '='.
                            if answer == '='                    
                                fwrite(h.portObject,' ');       % Send <space> command: Utility command 'Where Am I'.
                                answer = fread(h.portObject,1); % Reads answer, it should be 'B', communicating in Intelligent mode with the BOOT program.
                            
                                if answer(1) ~= 'B'
                                    h.errorDetected = true;
                                    disp([h.MSGERROR h.CLASSNAME functionName ': Forced re-boot did not occured.']);
                                    disp('   If error persists, use original software to start device.');
                                    return
                                end
                            else
                                h.errorDetected = true;
                                disp([h.MSGERROR h.CLASSNAME functionName ': Device did not entre in BOOT program.']);
                                disp('   If error persists, use original software to start device.');
                                return
                            end
                        end
                        
                        % TRIAX 320 BOOT program
                        if answer(1) == 'B'                             % Communicating in Intelligent mode with the BOOT program.
                            h.isinitialized = false;                    % Flag to know if monochromator is initialized.
                            isinitialized   = false;                    % Output function flag to know if monochromator is initialized.
                            
                            flushinput(h.portObject);                   % Remove data from port input buffer.
                            fprintf(h.portObject,['O2000' char(0)]);    % Send "O2000<null>" command: Utility command 'Start Main program'.
                            pause(.5);                                  % Recomended wait time provided in the manual.
                            answer = fread(h.portObject,1);             % Read answer, send '*' after transition to Main Program.
                            
                            if answer(1) == '*'
                                fwrite(h.portObject,' ');               % Send <space> command: Utility command 'Where Am I'.
                                answer = fread(h.portObject,1);         % Reads answer, it should be 'F', communicating in Intelligent mode with the MAIN program.
                            else
                                h.errorDetected = true;
                                disp([h.MSGERROR h.CLASSNAME functionName ': Device did not entre in MAIN program.']);
                                disp('   If error persists, use original software to start device.');
                                return
                            end
                        end
                        
                        fclose(h.portObject);                           % Close the serial port connection.
                        
                        % TRIAX 320 intelligent communication to the MAIN program
                        if answer(1) == 'F'
                            if isempty(h.isinitialized)                 % h.isinitialized is only empty if the <space> command resulted in 'F' answer (monochromator was already used before).
                                 h.isinitialized = true;                % Flag to know if monochromator is initialized.
                                 isinitialized   = true;                % Output function flag to know if monochromator is initialized.
                            end
                            
                            h.portObject = serial(port,...            	% Create serial port object.
                                            'Baudrate',baudRate,...
                                            'DataBits',dataBits,...
                                            'StopBits',stopBits,...
                                            'Parity',parity,...
                                            'Terminator','CR',...       % Some commands have termination 'CR others don't have termination.
                                            'TimeOut',h.TIMEOUT);
                                        
                            flushoutput(h.portObject);                 	% Remove data from output buffer.
                            flushinput(h.portObject);                   % Remove data from input buffer.
                            fopen(h.portObject);                        % Open the serial port connection.
                        end
                        
                        % Update Initialized Statuts
                        h.isconnected = true;
                        
                    case 'gpib'
                        vendor  = h.deviceInfo.gpib.vendor;                 % gpib adapter vendor
                        boardindex = h.deviceInfo.gpib.boardindex;          % controller board index.
                        primaryAddress = h.deviceInfo.gpib.primaryAddress;  % device gpib address
                         
                        h.portObject = gpib(vendor,boardindex,primaryAddress);  % Create gpib objet.
                        
                        flushoutput(h.portObject);                      % Remove data from port output buffer.
                        flushinput(h.portObject);                       % Remove data from port input buffer.
                        fopen(h.portObject);                            % Open the serial port connection.
                        
                        % Find Monochromator Status
                        for i = 1:10                                    % When powered-up, the monochromator could need several tries to detect serial port baundrate (in general work in the second try).
                            fwrite(h.portObject,' ');                   % Send <space> command: Utility command 'Where Am I'.
                            answer = fread(h.portObject,1);             % Reads 1st character send by the monochromator.
                            pause(h.TIMEREAD);                          % Waits for other characters.
                            flushinput(h.portObject);                   % flush buffer to remove characters.
                            
                            if answer(1) == 'B' || answer(1) == 'F' 
                                break;
                            else
                                fwrite(h.portObject,222);               % Send <222> binary character command: Utility command 'Re-boot If Hung'.
                                pause(0.2);                             % Recomended wait time provided in the manual.
                            end
                            
                            if i == 10                                  % Something is not working
                                h.errorDetected = true;                 % error detected
                                disp([h.MSGERROR h.CLASSNAME functionName ': Device did not entre in remote mode.']);
                                disp('   Try to power-down and power-up device.');
                                disp('   If error persists, use original software to start device.');
                                return
                            end
                        end
                        
                        % TRIAX 320 BOOT program
                        if answer(1) == 'B'                             % Communicating in Intelligent mode with the BOOT program.
                            h.isinitialized = false;                    % Flag to know if monochromator is initialized.
                            isinitialized   = false;                    % Output function flag to know if monochromator is initialized.
                            
                            flushinput(h.portObject);                   % Remove data from port input buffer.
                            fprintf(h.portObject,['O2000' char(0)]);    % Send "O2000<null>" command: Utility command 'Start Main program'.
                            pause(.5);                                  % Recomended wait time provided in the manual.
                            answer = fread(h.portObject,1);             % Read answer, send '*' after transition to Main Program.
                            
                            if answer(1) == '*'
                                fwrite(h.portObject,' ');               % Send <space> command: Utility command 'Where Am I'.
                                answer = fread(h.portObject,1);         % Reads answer, it should be 'F', communicating in Intelligent mode with the MAIN program.
                            else
                                h.errorDetected = true;
                                disp([h.MSGERROR h.CLASSNAME functionName ': Device did not entre in MAIN program.']);
                                disp('   If error persists, use original software to start device.');
                                return
                            end
                        end
                        
                        % TRIAX 320 intelligent communication to the MAIN program
                        if answer(1) == 'F'
                            if isempty(h.isinitialized)                 % h.isinitialized is only empty if the <space> command resulted in 'F' answer (monochromator was already used before).
                                 h.isinitialized = true;                % Flag to know if monochromator is initialized.
                                 isinitialized   = true;                % Output function flag to know if monochromator is initialized.
                            end
                        end
                         
                        % Update Initialized Statuts
                        h.isconnected = true;
                        
                    otherwise
                        h.errorDetected = true;
                        disp([h.MSGERROR h.CLASSNAME functionName ': Port choise invalid.']);
                end
            end
            
        end
        
        % =================================================================
        % FUNCTION: DISCONNECT
        function disconnect(h) % Disconnect device     
            
            if h.isconnected
            	fclose(h.portObject);       % Close connection.
                h.isconnected = false;      % Flag device not connected.
            end    
            
        end
        
        % =================================================================
        % FUNCTION: INITIALIZE MOTORS
        function initializeMotors(h)
            
            functionName    = 'connect()';      % Function name string to use in messages.
            h.errorDetected = false;            % Reset errorDetected flag.
            
            if ~h.isconnected
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Monochromator not connected.']);
                return
            end
            
            if ~h.isinitialized
                
                if h.HIGHMOTORTORQUEFLAG
                    % Gets Current Motor Speed
                    flushinput(h.portObject);                                   % Remove data from port input buffer.
                    fprintf(h.portObject,'C0');                                 % Send "C..." command: Motor command 'Get Speed'.
                    answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
                    checkError(h,answer);                                       % Function to check command error.
                    if ~h.errorDetected
                        motorSpeed = fscanf(h.portObject);                      % Reads answer, actual motor speed.
                        
                        % Sets Minimum Motor Speed to Increase Torque
                        flushinput(h.portObject);                               % Remove data from port input buffer.
                        fprintf(h.portObject,['B0,' h.HIGHMOTORTORQUESPEED]);   % Send "B..." command: Motor command 'Set Speed'.
                        answer = fread(h.portObject,1);                         % Reads answer, it should be 'o'.
                        checkError(h,answer);                                   % Function to check command error.
                        if ~h.errorDetected

                            % Initialize Monochromator Motors
                            h.portObject.TimeOut = 300;                         % Change time out for port object.
                            flushinput(h.portObject);                           % Remove data from port input buffer.
                            fprintf(h.portObject,'A');                          % Send "A" command: Motor command 'Initialize Monochromator motors'.
                            answer = fread(h.portObject,1);                     % Reads answer, it should be 'o', sent only after end of initialization.
                            checkError(h,answer);                              	% Function to check command error.
                            h.portObject.TimeOut = h.TIMEOUT;                   % Change time out back to previous value.
                            
                            % Restores Motor Speed
                            fprintf(h.portObject,['B0,' motorSpeed]);           % Send "B..." command: Motor command 'Set Speed'.
                            answer = fread(h.portObject,1);                     % Reads answer, it should be 'o'.
                            checkError(h,answer);                              	% Function to check command error.
                        else
                            h.errorDetected = true;
                            disp([h.MSGERROR h.CLASSNAME functionName ': Communication error occured.']);
                            return
                        end
                    else
                        h.errorDetected = true;
                        disp([h.MSGERROR h.CLASSNAME functionName ': Communication error occured.']);
                        return
                    end
                else
                    % Initialize Monochromator Motors
                    h.portObject.TimeOut = 200;                         % Change time out for port object.
                    flushinput(h.portObject);                           % Remove data from port input buffer.
                    fprintf(h.portObject,'A');                          % Send "A" command: Motor command 'Initialize Monochromator motors'.
                    answer = fread(h.portObject,1);                     % Reads answer, it should be 'o', sent only after end of initialization.
                    checkError(h,answer);                              	% Function to check command error.
                    h.portObject.TimeOut = h.TIMEOUT;                   % Change time out back to previous value.
                end
            end
            
            h.isinitialized = true;                         % Flag monochromator motors initialized.
            
        end
        
        % =================================================================
        % FUNCTION: SET WAVELENGTH POSITION
        function setWavelengthPosition(h,val)
            
            functionName    = 'setWavelengthPosition(wavelength)';	% Function name string to use in messages.
            h.errorDetected = false;                                % Reset errorDetected flag.
            
            % Calculate Grating Base Factor
            if isempty(h.gratingBaseFactor)
                h.calculateGratingBaseFactor();                     % Calculate grating base factor.
            end
                 
            % Check if is Only One Number
            if ~isscalar(val) || ~isreal(val)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Wavelength value invalid.']);
                return
            end

            % Check if is Inside Monochromator Parameters
            if val >= (h.GRATINGMOTORMAXLIMIT / h.gratingBaseFactor) && val <= (h.GRATINGMOTORMINLIMIT / h.gratingBaseFactor)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Wavelength out of range.']);
                return
            end
            
            % Correct Wavelength Value
            newWavelength = round(val * h.gratingBaseFactor,4);
            
            % Set New Wavelength
            flushinput(h.portObject);                                   % Remove data from port input buffer.
            fprintf(h.portObject,['Z60,0,' num2str(newWavelength)]);    % Send "Z60..." command: Triax command 'Set wavelength'.
            answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
            checkError(h,answer);                                       % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: GET WAVELENGTH POSITION
        function wavelength = getWavelengthPosition(h)
            
            h.errorDetected = false;                        % Reset errorDetected flag.
            
            % Calculate Grating Base Factor
            if isempty(h.gratingBaseFactor)
                h.calculateGratingBaseFactor();         % Calculate grating base factor.
            end
            
            % Get Grating Position
            flushinput(h.portObject);                   % Remove data from port input buffer.
            fprintf(h.portObject,'Z62,0');              % Send "Z62..." command: Triax command 'Get wavelength'.
            answer = fread(h.portObject,1);             % Reads answer, it should be 'o'.
            checkError(h,answer);                       % Function to check command error.
            answer = str2double(fscanf(h.portObject)); 	% Reads answer, wavelength + <CR>.
            wavelength = answer / h.gratingBaseFactor;  % Grating position corresponds to 2nd character.
            
        end
        
        % =================================================================
        % FUNCTION: MOVETO
        % Input: val, wavelength in nanometers.
        function moveTo(h,val)
            
            functionName    = 'moveTo(wavelength)';         % Function name string to use in messages.
            h.errorDetected = false;                        % Reset errorDetected flag.
                       
            % Calculate Grating Base Factor
            if isempty(h.gratingBaseFactor)
                h.calculateGratingBaseFactor();             % Calculate grating base factor.
            end
                 
            % Check if is Only One Number
            if ~isscalar(val) || ~isreal(val)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Wavelength value invalid.']);
                return
            end

            % Check if is Inside Monochromator Parameters
            if val >= (h.GRATINGMOTORMAXLIMIT / h.gratingBaseFactor) && val <= (h.GRATINGMOTORMINLIMIT / h.gratingBaseFactor)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Wavelength out of range.']);
                return
            end

            % Correct Wavelength Value
            newWavelength = round((val * h.gratingBaseFactor),4);
            
            % Move Motor - Change Wavelength
            flushinput(h.portObject);                                   % Remove data from port input buffer.
            fprintf(h.portObject,['Z61,0,' num2str(newWavelength)]);	% Send "Z61..." command: Triax command 'Move to wavelength'.
            answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
            checkError(h,answer);                                       % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: SET GRATING MOTOR SPEED
        % Inputs: minSpeed, minimum frequency in Hz (steps per second, Recomended 1000 Hz).
        %         maxSpeed, maximum frequency in Hz (speps per second, Recomended 4000 Hz).
        %         riseTime, rise time between minimum and maximum frequency in miliseconds (Recomended 250 ms).
        function setMotorSpeed(h,minSpeed,maxSpeed,riseTime)
            
            functionName    = 'setMotorSpeed(minSpeed,maxSpeed,riseTime)';     	% Function name string to use in messages.
            h.errorDetected = false;                                            % Reset errorDetected flag.
            
            % Check Minimum Speed
                % Check if is Only One Number
                if ~isscalar(minSpeed) || ~isreal(minSpeed)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Turret motor minimum steps/s invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if minSpeed < h.GRATINGMOTORMINSPEED || minSpeed > h.GRATINGMOTORMAXSPEED              
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Turret minimum steps/s out of range.']);
                    return
                end

                % Round Value
                minSpeed = round(minSpeed);
            
            % Check Maximum Speed
                % Check if is Only One Number
                if ~isscalar(maxSpeed) || ~isreal(maxSpeed)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Turret motor maximum steps/s invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if maxSpeed > h.GRATINGMOTORMAXSPEED || maxSpeed < minSpeed
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Turret maximum steps/s out of range.']);
                    return
                end

                % Round Value
                maxSpeed = round(maxSpeed);
            
            % Check Rise Time
                % Check if is Only One Number
                if ~isscalar(riseTime) && isreal(riseTime)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Turret motor rise time invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if riseTime > h.GRATINGMOTORMAXRISETIME
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Turret rise time out of range.']);
                    return
                end

                % Round Value
                riseTime = round(riseTime);
            
            % Set Motor Spped
            flushinput(h.portObject);                   % Remove data from port input buffer.
            fprintf(h.portObject,['B0,' minSpeed ',' maxSpeed ',' riseTime]);   % Send "B..." command: Motor command 'Set Speed'.
            answer = fread(h.portObject,1);             % Reads answer, it should be 'o'.
            checkError(h,answer);                       % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: GET GRATING MOTOR SPEED
        % Outputs: minSpeed, minimum frequency in Hz (steps per second).
        %          maxSpeed, maximum frequency in Hz (speps per second).
        %          riseTime, rise time between minimum and maximum frequency in miliseconds.
        function [minSpeed,maxSpeed,riseTime] = getMotorSpeed(h)
            
            h.errorDetected = false;                        % Reset errorDetected flag.
            
            % Get Motor Spped
            flushinput(h.portObject);                       % Remove data from port input buffer.
            fprintf(h.portObject,'C0');                     % Send "C..." command: Motor command 'Get Speed'.
            answer = fread(h.portObject,1);                 % Reads answer, it should be 'o'.
            checkError(h,answer);                           % Function to check command error.
            
            if ~h.errorDetected
                answer = fscanf(h.portObject);              % Reads answer, it should be 'minSpeed,maxSpeed,riseTime'.

                % Obtaining Speed and Risetime Values
                commas = strfind(answer,',');               % Find sparation commas.

                minSpeed = answer(1:commas(1)-1);           % Turret motor minimum frequency in Hz.
                maxSpeed = answer(commas(1)+1:commas(2)-1); % Turret motor maxinum frequency in Hz.
                riseTime = answer(commas(2)+1:end);         % Rise time from minimum to maximum frequency in miliseconds.
            end
            
        end
        
        % =================================================================
        % FUNCTION: SET GRATING POSITION INDEX
        % Input: val = 0, 1 or 2 (0 = 1st turret position, 1 = 2nd turret position, 2 = 3rd turret position)
        % When the monochromator is re-started (turn-off / turn-on), the turret grating index is  equal to zero.
        % It is not possible set the index value with initialize motors. This function allows the user to set
        % the turret index inside the class object without initialize motors. This can be useful when the index
        % is known and the monochromator is reguraly turn-off, avoiding the long motor initialization process.
        function setGratingIndex(h,val)
            
            functionName    = 'setGratingIndex(index)';     	% Function name string to use in messages.
            h.errorDetected = false;                            % Reset errorDetected flag.
            
            % Check if is Only One Number
            if ~isscalar(val) || ~isreal(val)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Turret grating index invalid.']);
                return
            end

            % Check if is Inside Monochromator Parameters
            if val < 0 && val > 2
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Turret grating index invalid.']);
                return
            end
            
            % Set Grating Position Index
            h.gratingIndex = val;
            
        end
        
        % =================================================================
        % FUNCTION: GET GRATING POSITION INDEX
        % Output: gratingPosition = 0, 1 or 2 (0 = 1st turret position, 1 = 2nd turret position, 2 = 3rd turret position)
        function gratingPosition = getGratingIndex(h)
            
            h.errorDetected = false;                        % Reset errorDetected flag.
            
            % Get Turret Grating Index
            if isempty(h.gratingIndex)
                flushinput(h.portObject);                   % Remove data from port input buffer.
                fprintf(h.portObject,'Z452,0,0,0');         % Send "Z452..." command: Triax command 'Read Index Turret Position'.
                answer = fscanf(h.portObject);            	% Reads answer, it should be 'o' + grating position + <CR>.
                gratingPosition = str2double(answer(2));    % Grating position corresponds to 2nd character.
                h.gratingIndex = gratingPosition;
            else
                gratingPosition = h.gratingIndex;
            end
            
        end
        
        % =================================================================
        % FUNCTION: MOVE GRATING POSITION
        % Inputs: val = 0, 1 or 2 (0 = 1st turret position, 1 = 2nd turret position, 2 = 3rd turret position)
        function moveGratingIndex(h,val)
            
            functionName    = 'moveGratingIndex(index)';                        % Function name string to use in messages.
            h.errorDetected = false;                                            % Reset errorDetected flag.
            
            % Check if is Only One Number
            if ~isscalar(val) || ~isreal(val)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Turret grating index invalid.']);
                return
            end

            % Check if is Inside Monochromator Parameters
            if val < 0 && val > 2
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Turret grating index invalid.']);
                return
            end
            
            % Round Value
            desiredGratingIndex = fix(val);
            
            % Correct index - in case index was set with function setGratingIndex()
            h.gratingIndex = h.getGratingIndex();
            % realPositionVsNewDesiredPosition = [0 1 2;...
            %                                     2 0 1;...
            %                                     1 2 0];
            % newGratingIndex = realPositionVsNewDesiredPosition(h.gratingIndex+1,desiredGratingIndex+1);
            newGratingIndex = h.gratingIndex;
            
            % Change Grating
            if h.HIGHMOTORTORQUEFLAG
                % Gets Current Motor Speed
                flushoutput(h.portObject);                                      % Remove data from port output buffer.
                flushinput(h.portObject);                                       % Remove data from port input buffer.
                fprintf(h.portObject,'C0');                                     % Send "C..." command: Motor command 'Get Speed'.
                answer = fread(h.portObject,1);                                 % Reads answer, it should be 'o'.
                checkError(h,answer);                                           % Function to check command error.
                if ~h.errorDetected
                    motorSpeed = fscanf(h.portObject);                          % Reads answer, actual motor speed.

                    % Sets Minimum Motor Speed to Increase Torque
                    flushinput(h.portObject);                                   % Remove data from port input buffer.
                    fprintf(h.portObject,['B0,' h.HIGHMOTORTORQUESPEED]);       % Send "B..." command: Motor command 'Set Speed'.
                    answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
                    checkError(h,answer);                                       % Function to check command error.
                    if ~h.errorDetected

                        % Set New Grating Turret Position
                        flushinput(h.portObject);                               % Remove data from port input buffer.
                        fprintf(h.portObject,['Z451,0,0,0,' num2str(newGratingIndex)]);  % Send "Z451..." command: Triax command 'Set Index Device'.
                        answer = fread(h.portObject,1);                         % Reads answer, it should be 'o'.
                        checkError(h,answer);                                   % Function to check command error.
                        
                        % Wait until Turret Motor Stops
                        while answer ~= 'z'
                            flushinput(h.portObject);                           % Remove data from port input buffer.
                            fprintf(h.portObject,'E');                          % Send "E" command: Motor command 'Motor Busy status'.
                            answer = fread(h.portObject,1);                     % Read answer, it should be 'o' confirming command.
                            answer = fread(h.portObject,1);                     % Read answer, it should be 'z' when motor is stopped.
                            pause(1);                                           % Wait 1 second until question again turret movement status.
                        end

                        % Restores Motor Speed
                        flushinput(h.portObject);                               % Remove data from port input buffer.
                        fprintf(h.portObject,['B0,' motorSpeed]);               % Send "B..." command: Motor command 'Set Speed'.
                        answer = fread(h.portObject,1);                         % Reads answer, it should be 'o'.
                        checkError(h,answer);                                   % Function to check command error.
                    end

                end
            else
                % Set New Grating Turret Position
                flushinput(h.portObject);                                       % Remove data from port input buffer.
                fprintf(h.portObject,['Z451,0,0,0,' num2str(newGratingIndex)]);	% Send "Z451..." command: Triax command 'Set Index Device'.
                answer = fread(h.portObject,1);                                 % Reads answer, it should be 'o'.
                checkError(h,answer);                                           % Function to check command error.

                % Wait until Turret Motor Stops
                while answer ~= 'z'
                    flushinput(h.portObject);                                   % Remove data from port input buffer.
                    fprintf(h.portObject,'Z453');                               % Send "Z453" command: Triax command 'Index device status'.
                    answer = fread(h.portObject,1);                             % Read answer, it should be 'o' confirming command.
                    answer = fread(h.portObject,1);                             % Read answer, it should be 'z' when motor is stopped.
                    pause(1);                                                   % Wait 1 second until question again turret movement status.
                end
            end
            
            h.gratingIndex = desiredGratingIndex;                           	% Actualize to new turret grating index position.
            h.calculateGratingBaseFactor();                                     % Calculate grating base factor.
           
        end
        
        % =================================================================
        % FUNCTION: SET GRATING BASE FACTOR
        % Monochromator base value = 1200 gr/mm
        % for ex.: val = (600 gr/mm) / 1200
        function setGratingBaseFactor(h, val)
            
            functionName    = 'setGratingBaseFactor(groveDensity)';            % Function name string to use in messages.
            h.errorDetected = false;                                            % Reset errorDetected flag.
            
            % Check if is Only One Number
            if ~isscalar(val) || ~isreal(val)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Grating gr/mm invalid.']);
                return
            end

            % Check if is Inside Monochromator Parameters
            if val <= 60 && val >= 3600
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Grating gr/mm invalid.']);
                return
            end
            
            % Round Value
            groveDensity = fix(val);
            
            % Set Grating Position Index
            h.gratingBaseFactor = groveDensity / h.BASEGRATING;
            
        end
        
        % =================================================================
        % FUNCTION: GET GRATING BASE FACTOR
        function baseFactor = getGratingBaseFactor(h)
            
            baseFactor = h.gratingBaseFactor;
            
        end
        
        % =================================================================
        % FUNCTION: SET SLIT + MOTOR SPEED
        % Inputs: motorSpeed, frequency in Hz (steps per second, Recomended 300 Hz).
        function setSlitMotorSpeed(h,slitNumber,motorSpeed)
            
            functionName    = 'setSlitMotorSpeed(slitNumber,motorSpeed)';  	% Function name string to use in messages.
            h.errorDetected = false;                                        % Reset errorDetected flag.
            
            % Check Slit Number Input
                % Check if is Only One Number
                if ~isscalar(slitNumber) || ~isreal(slitNumber)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if slitNumber < 0 && slitNumber > 3
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end
            
                % Rough value
                slitNumber = fix(slitNumber);
                
            % Check Motor Speed
                % Check if is Only One Number
                if ~isscalar(motorSpeed) || ~isreal(motorSpeed)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit motor speed invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if motorSpeed >= h.SLITMOTORMINSPEED && motorSpeed <= h.SLITMOTORMAXSPEED              
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit motor speed out of range.']);
                    return
                end

                % Convert to String
                motorSpeed = round(motorSpeed);
            
            % Set Motor Speed
            flushinput(h.portObject);                                   % Remove data from port input buffer.
            fprintf(h.portObject,['g0,' num2str(slitNumber) ',' num2str(motorSpeed)]);   % Send "g..." command: SLIT command 'Set Speed'.
            answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
            checkError(h,answer);                                       % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: GET GRATING MOTOR SPEED
        % Outputs: minSpeed, minimum frequency in Hz (steps per second).
        %          maxSpeed, maximum frequency in Hz (speps per second).
        %          riseTime, rise time between minimum and maximum frequency in miliseconds.
        function motorSpeed = getSlitMotorSpeed(h,slitNumber)
            
            functionName    = 'getSlitMotorSpeed(slitNumber)';  	% Function name string to use in messages.
            h.errorDetected = false;                                % Reset errorDetected flag.
            
            % Check Slit Number Input
                % Check if is Only One Number
                if ~isscalar(slitNumber) || ~isreal(slitNumber)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if slitNumber < 0 && slitNumber > 3
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end
            
                % Rough value
                slitNumber = fix(slitNumber);
            
            % Get Motor Spped
            flushinput(h.portObject);                               % Remove data from port input buffer.
            fprintf(h.portObject,['h0,' num2str(slitNumber)]);      % Send "h..." command: Slit command 'Read Speed'.
            answer = fread(h.portObject,1);                         % Reads answer, it should be 'o'.
            checkError(h,answer);                                   % Function to check command error.
            if h.errorDetected
                return
            end
            answer = fscanf(h.portObject);                          % Reads answer, it should be 'minSpeed,maxSpeed,riseTime'.
            
            % Obtaining Motor Speed
            motorSpeed = str2double(answer);                        % Turret motor minimum frequency in Hz.
           
        end
        
        % =================================================================
        % FUNCTION: SET SLIT WIDTH
        function setSlitWidth(h,slitNumber,width)
            
            functionName    = 'setSlitWidth(slitNumber,width)';  	% Function name string to use in messages.
            h.errorDetected = false;                                % Reset errorDetected flag.
            
            % Check Slit Number Input
                % Check if is Only One Number
                if ~isscalar(slitNumber) || ~isreal(slitNumber)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if slitNumber < 0 && slitNumber > 3
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end
            
                % Round value
                slitNumber = fix(slitNumber);
            
            % Check Width Number Input
                % Check if is Only One Number
                if (~isscalar(width) || ~isreal(width)) && ~contains(lower(width),'max')
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit width invalid.']);
                    return
                end
                if ischar(width)
                    width = h.SLITAPERTUREMAX;
                end

                % Check if is Inside Monochromator Parameters
                if width < 0 && width > h.SLITAPERTUREMAX
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit width out of range.']);
                    return
                end
            
                % Round value
                widthSteps = round( width * h.SLITCONVERTIONFACTOR );
            
            % Set New Wavelength
            flushinput(h.portObject);                                   % Remove data from port input buffer.
            fprintf(h.portObject,['i0,' num2str(slitNumber) ',' num2str(widthSteps)]);    % Send "i..." command: Slit command 'Set Position'.
            answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
            checkError(h,answer);                                       % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: GET SLIT WIDTH
        function slitAperture = getSlitWidth(h,slitNumber)
            
            functionName    = 'getSlitWidth(slitNumber)';               % Function name string to use in messages.
            h.errorDetected = false;                                    % Reset errorDetected flag.
            
            % Check if is Only One Number
            if ~isscalar(slitNumber) || ~isreal(slitNumber)
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                return
            end

            % Check if is Inside Monochromator Parameters
            if slitNumber < 0 && slitNumber > 3
                h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                return
            end

            % Rough value
            slitNumber = fix(slitNumber);
            
            % Get Slit Position
            flushinput(h.portObject);                                   % Remove data from port input buffer.
            fprintf(h.portObject,['j0,' num2str(slitNumber)]);          % Send "j..." command: Slit command 'Read Position'.
            answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
            checkError(h,answer);                                       % Function to check command error.
            answer = str2double(fscanf(h.portObject));                  % Reads answer, motor number of steps + <CR>.
            slitAperture = round(answer / h.SLITCONVERTIONFACTOR,2);    % Slit aperture in milimeters.
            
        end

        % =================================================================
        % FUNCTION: MOVE SLIT WIDTH
        % Inputs: SlitNumber = 0,1,2 or 3 (0 = Front Entrance Slit; 1 = Side Entrance Slit; 2 = Front Exit Slit; 3 = Side Exit Slit)
        %         Width in milimeters or 'max' for maximum slit width
        function moveSlitWidth(h,slitNumber,width)
            
            functionName    = 'moveSlitWidth(slitNumber,width)';     	% Function name string to use in messages.
            h.errorDetected = false;                                    % Reset errorDetected flag.
            
            % Check Slit Number Input
                % Check if is Only One Number
                if ~isscalar(slitNumber) || ~isreal(slitNumber)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if slitNumber < 0 && slitNumber > 3
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Rough value
                slitNumber = fix(slitNumber);
            
            % Check Width Number Input
                % Check if is Only One Number
                if (~isscalar(width) || ~isreal(width)) && ~contains(lower(width),'max')
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit width invalid.']);
                    return
                end
                if ischar(width)
                    width = h.SLITAPERTUREMAX;
                end

                % Check if is Inside Monochromator Parameters
                if width < 0 && width > h.SLITAPERTUREMAX
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit width out of range.']);
                    return
                end

            % Calculate Relative Movement of the Slit
            currentSlitAperture = h.getSlitWidth(slitNumber);           % Get current slit width in milimeters.
            relativeWidth = width - currentSlitAperture;                % Calculate width difference.
            widthSteps = round(relativeWidth * h.SLITCONVERTIONFACTOR); % Number of motor steps.
                
            % Set New Slit Width
            flushinput(h.portObject);                                   % Remove data from port input buffer.
            fprintf(h.portObject,['k0,' num2str(slitNumber) ',' num2str(widthSteps)]);    % Send "k..." command: Slit command 'Move relative'.
            pause(widthSteps/h.SLITMOTORSPEED);                         % Wait time for slit motor to stop.
            answer = fread(h.portObject,1);                             % Reads answer, it should be 'o'.
            checkError(h,answer);                                       % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: SET SLIT BANDWIDTH
        % Inputs: SlitNumber = 0,1,2 or 3 (0 = Front Entrance Slit; 1 = Side Entrance Slit; 2 = Front Exit Slit; 3 = Side Exit Slit)
        %         bandwidth in nanometers or 'max' for maximum slit width
        function setSlitBandwidth(h,slitNumber,bandwidth)
            
            functionName    = 'setSlitBandwidth(slitNumber,bandwidth)';     % Function name string to use in messages.
            h.errorDetected = false;                                        % Reset errorDetected flag.
            
            % Check Slit Number Input
                % Check if is Only One Number
                if ~isscalar(slitNumber) || ~isreal(slitNumber)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if slitNumber < 0 && slitNumber > 3
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Rough value
                slitNumber = fix(slitNumber);
            
            % Check Bandwidth Number Input
                % Check if is Only One Number
                if (~isscalar(bandwidth) || ~isreal(bandwidth)) && ~contains(lower(bandwidth),'max')
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit bandwidth invalid.']);
                    return
                end
                if ischar(bandwidth)
                    bandwidth = h.SLITAPERTUREMAX * h.SPECTRALDISPERSION;
                end

                % Check if is Inside Monochromator Parameters
                if bandwidth < 0 && bandwidth > (h.SLITAPERTUREMAX * h.SPECTRALDISPERSION)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit bandwidth out of range.']);
                    return
                end
            
            % Calculate Slit Width in Milimeters
            width = bandwidth / h.SPECTRALDISPERSION;
            
            % Move Silt Witdth
            h.moveSlitWidth(h,slitNumber,width);
        end
        
        % =================================================================
        % FUNCTION: GET SLIT BANDWIDTH
        % Inputs: SlitNumber = 0,1,2 or 3 (0 = Front Entrance Slit; 1 = Side Entrance Slit; 2 = Front Exit Slit; 3 = Side Exit Slit)
        % Outputs: spetral bandwidth in nanometers.
        function slitBandwidth = getSlitBandwidth(h,slitNumber)
            
            functionName    = 'getSlitBandwidth(slitNumber)';     	% Function name string to use in messages.
            h.errorDetected = false;                                % Reset errorDetected flag.
            
            % Check Slit Number Input
                % Check if is Only One Number
                if ~isscalar(slitNumber) || ~isreal(slitNumber)
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Check if is Inside Monochromator Parameters
                if slitNumber < 0 && slitNumber > 3
                    h.errorDetected = true;
                    disp([h.MSGERROR h.CLASSNAME functionName ': Slit number index invalid.']);
                    return
                end

                % Rough value
                slitNumber = fix(slitNumber);
            
            % Get Slit Width and Calculate Bandwidth
            slitBandwidth = h.getSlitWidth(h,slitNumber) * h.SPECTRALDISPERSION;
        end
        
        % =================================================================
        % FUNCTION: ENTRACE MIRROR SIDE
        function entranceMirrorSide(h)
            
            h.errorDetected = false;            % Reset errorDetected flag.
            
            % Move Entrace Mirror to Side
            flushinput(h.portObject);          	% Remove data from port input buffer.
            fprintf(h.portObject,'c0');         % Send "c..." command: Accessory command 'Entrace Mirror Side'.
            pause(h.TIMEMOVEMIRROR);            % Wait time to move the entrance mirror.
            answer = fread(h.portObject,1);     % Reads answer, it should be 'o'.
            checkError(h,answer);               % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: ENTRACE MIRROR FRONT
        function entranceMirrorFront(h)
            
            h.errorDetected = false;            % Reset errorDetected flag.
            
            % Move Entrace Mirror to Side
            flushinput(h.portObject);          	% Remove data from port input buffer.
            fprintf(h.portObject,'d0');         % Send "d..." command: Accessory command 'Entrace Mirror Front'.
            pause(h.TIMEMOVEMIRROR);            % Wait time to move the entrance mirror.
            answer = fread(h.portObject,1);     % Reads answer, it should be 'o'.
            checkError(h,answer);               % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: EXIT MIRROR SIDE
        function exitMirrorSide(h)
            
            h.errorDetected = false;            % Reset errorDetected flag.
            
            % Move Entrace Mirror to Side
            flushinput(h.portObject);          	% Remove data from port input buffer.
            fprintf(h.portObject,'e0');         % Send "e..." command: Accessory command 'Exit Mirror Side'.
            pause(h.TIMEMOVEMIRROR);            % Wait time to move the entrance mirror.
            answer = fread(h.portObject,1);     % Reads answer, it should be 'o'.
            checkError(h,answer);               % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: EXIT MIRROR FRONT
        function exitMirrorFront(h)
            
            h.errorDetected = false;            % Reset errorDetected flag.

            % Move Entrace Mirror to Side
            flushinput(h.portObject);          	% Remove data from port input buffer.
            fprintf(h.portObject,'f0');         % Send "f..." command: Accessory command 'Exit Mirror Front'.
            answer = fread(h.portObject,1);     % Reads answer, it should be 'o'.
            checkError(h,answer);               % Function to check command error.
            pause(h.TIMEMOVEMIRROR);            % Wait time to move the entrance mirror.
            
        end
        
        % =================================================================
        % FUNCTION: OPEN SHUTTER
        function openShutter(h)
            
            h.errorDetected = false;            % Reset errorDetected flag.
            
            % Move Entrace Mirror to Side
            flushinput(h.portObject);          	% Remove data from port input buffer.
            fprintf(h.portObject,'W0');         % Send "W..." command: Accessory command 'Shutter Open'.
            pause(h.TIMEMOVESHUTTER);           % Wait time to move the entrance mirror.
            answer = fread(h.portObject,1);     % Reads answer, it should be 'o'.
            checkError(h,answer);               % Function to check command error.
            
        end
        
        % =================================================================
        % FUNCTION: CLOSE SHUTTER
        function closeShutter(h)
            
            h.errorDetected = false;            % Reset errorDetected flag.
            
            % Move Entrace Mirror to Side
            flushinput(h.portObject);          	% Remove data from port input buffer.
            fprintf(h.portObject,'X0');         % Send "X..." command: Accessory command 'Shutter Close'.
            pause(h.TIMEMOVESHUTTER);           % Wait time to move the entrance mirror.
            answer = fread(h.portObject,1);     % Reads answer, it should be 'o'.
            checkError(h,answer);               % Function to check command error.
            
        end
        
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M E T H O D S - DEPENDENT, REQUIRE SET/GET
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        
        % =================================================================
        % FUNCTION: SET ERROR DETECTED
        function set.errorDetected(h,errorDetected)
            
            h.errorDetected = errorDetected;
            
        end

        % =================================================================
        % FUNCTION: GET ERROR DETECTED
        function errorDetected = get.errorDetected(h)
            
            errorDetected = h.errorDetected;
            
        end
        
    end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M E T H O D S (Hidden) - INTERNAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods (Hidden)
        
        % =================================================================
        % FUNCTION: CHECK COMMAND ERROR
        function checkError(h,answer)
            
            functionName    = 'checkError(answer)';     % Function name string to use in messages.
            
            if answer ~= 'o'
            	h.errorDetected = true;
                disp([h.MSGERROR h.CLASSNAME functionName ': Command error.']);
            end
            
        end
        
        % =================================================================
        % FUNCTION: CALCULATE GRATING BASE FACTOR
        function calculateGratingBaseFactor(h)
            
            if isempty(h.gratingIndex)
                h.gratingIndex = getGratingIndex(h);       % Get turret index position.
            end
            
            switch h.gratingIndex
                case 0
                    h.gratingBaseFactor = h.GRATING_0 / h.BASEGRATING;
                case 1
                    h.gratingBaseFactor = h.GRATING_1 / h.BASEGRATING;
                case 2
                    h.gratingBaseFactor = h.GRATING_2 / h.BASEGRATING;
            end
                
        end
        
    end
    
end