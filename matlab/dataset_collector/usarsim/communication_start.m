function [] = communication_start()
    global settings;

    % setting up connection
    Connection2UT = tcpip(settings.usarsim_ip, settings.usarsim_port);

    % setting the proper termination
    set(Connection2UT, 'Terminator', {'CR/LF','CR/LF'});
    
    % opening a connection
    open(Connection2UT);
    
    % checking for message or data
    fscanf(Connection2UT)
    ConStat = get(Connection2UT,'Status')
end