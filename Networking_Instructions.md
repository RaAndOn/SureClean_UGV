
Connect to CMU Device:
    Static IP address : 172.26.198.249

    Jetson Name : Nvidia

    A general SSH command : ssh nvidia@172.26.198.249

    Make sure the Jetson is connected to "CMU Device" (It is set as default)

    Should the jetson loose wi-fi connection in the elevator or outside, use joystick to control the Husky

    The other alternative is to use laptop and directly interface the Husky via the RS-232
    
    
 Connect to TP-Link_E7E8:
 
    Static IP : 192.168.0.126
    
    ssh nvidia@192.168.0.126
    
    Base RTk GPS:     192.168.0.100

  Husky Rover GPS : 192.168.0.102

  DJI Rover GPS :   192.168.0.101

  UAV Onboard computer : 192.168.0.125

  Nvidia Jetson :   192.168.0.126


To add new address : Connect to TP-Link_E7E8

    type the following in firefox/chrome : 192.168.0.1

    goto 'DHCP --> Address' from the left sidebar

    Click 'Add' and enter new MacID and Your desired IP starting with 192.168.0.XXX

    Make sure you enter an IP that is not already taken :)

    
    
