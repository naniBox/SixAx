@set PATH=%PATH%;"C:\Program Files (x86)\SEGGER\JLink_V512c\"
start "JLink" "C:\Program Files (x86)\SEGGER\JLink_V512c\JLinkGDBServer.exe" -if SWD -select USB -device STM32F103T8 -if SWD -speed auto
