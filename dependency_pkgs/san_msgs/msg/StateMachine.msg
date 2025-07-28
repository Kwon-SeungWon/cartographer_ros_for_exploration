# State
uint8 INIT           = 0x00      # 0
uint8 IDLE           = 0x10      # 16
uint8 AUTO           = 0x20      # 32
uint8 MANUAL         = 0x30      # 48
uint8 DOCKING        = 0x40      # 64
uint8 UNDOCKING      = 0x50      # 80  
uint8 CHARGING       = 0x60      # 96  
uint8 MANIPULATION   = 0x70      # 112  Send for Robot Arm
uint8 EMERGENCY      = 0x80      # 128
uint8 STOP           = 0x90      # 144
# uint8 SLAM           = 0x100     # 160 Back to the Docking Waypoint Node

uint8 state

# Status 
uint8 RUNNING   = 0   
uint8 SUCCESS   = 1  
uint8 FAIL      = 2      

uint8 status
