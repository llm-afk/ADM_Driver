MEMORY
{
PAGE 0 :
   RAMPRG      : origin = 0x008000, length = 0x001000  

   APP_SPACE   : origin = 0x3F0000, length = 0x006000    
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     
   BEGIN       : origin = 0x3F7FF6, length = 0x000002   
   CSM_PWL     : origin = 0x3F7FF8, length = 0x000008  

   OTP         : origin = 0x3D7800, length = 0x000400      

   IQTABLES    : origin = 0x3FE000, length = 0x000B50   
   IQTABLES2   : origin = 0x3FEB50, length = 0x00008C 
   IQTABLES3   : origin = 0x3FEBDC, length = 0x0000AA	  
   ROM         : origin = 0x3FF27C, length = 0x000D44 
   RESET       : origin = 0x3FFFC0, length = 0x000002    
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E    

PAGE 1 :                                          
   RAMM0       : origin = 0x000100, length = 0x000300   
   RAMM1       : origin = 0x000400, length = 0x000400    
   RAML0       : origin = 0x009000, length = 0x001000
   RAMH0       : origin = 0x00A000, length = 0x002000
}

 
SECTIONS
{
   .cinit              : > APP_SPACE    PAGE = 0 
   .pinit              : > APP_SPACE    PAGE = 0 
   .text               : > APP_SPACE    PAGE = 0 
   .binit              : > APP_SPACE    PAGE = 0
   .ovly               : > APP_SPACE    PAGE = 0
   codestart           : > BEGIN        PAGE = 0 
   ramfuncs            :
   {
		-lADP32F036_027_Flash_API_ZONE.lib(.econst)
		-lADP32F036_027_Flash_API_ZONE.lib(.text)
   }LOAD = APP_SPACE, RUN = RAMPRG, PAGE = 0, TABLE(_prginRAM)

   .stack              : > RAMM1        PAGE = 1
   .esysmem            : > RAML0        PAGE = 1
   .ebss               : >> RAML0|RAMH0 PAGE = 1

   .econst             : > APP_SPACE    PAGE = 0 
   .switch             : > APP_SPACE    PAGE = 0

   csmpasswds          : > CSM_PWL      PAGE = 0 
   csm_rsvd            : > CSM_RSVD     PAGE = 0 
                                               
   .reset              : > RESET        PAGE = 0, TYPE = DSECT
   vectors             : > VECTORS      PAGE = 0, TYPE = DSECT
}

