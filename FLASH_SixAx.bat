@set PATH=%PATH%;"C:\Program Files (x86)\SEGGER\JLink_V512c\"
c:
cd "c:\ChibiStudio\workspace30\SixAx\"
jlink -CommanderScript FLASH_SixAx.jlink
pause
