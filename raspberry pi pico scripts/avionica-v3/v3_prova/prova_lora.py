import time
import board
import busio
import digitalio

lora = busio.UART(board.GP16, board.GP17, baudrate=9600, bits=8, parity=None, stop=1, timeout=1)

if lora.in_waiting:
    message_received = lora.read(lora.in_waiting).decode()
    print(message_received)
    
lora.write(bytes("mammt wserdcfvgyhbnj szexdcftvgyhbnj sxcdftvygbuhnijmk,l","utf-8" ))
print("fatto")